// Library header
#include <DaliNZU.h>

//Helpers
#define DALI_BUS_LOW() digitalWrite(this->tx_pin,HIGH); this->tx_bus_low=1
#define DALI_BUS_HIGH() digitalWrite(this->tx_pin,LOW); this->tx_bus_low=0
#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin)==LOW)
#define DALI_IS_BUS_HIGH() (digitalRead(this->rx_pin)==HIGH)

#define DALI_BAUD 1200
// Te = half cycle = 416.67 �s +/- 10 %
#define DALI_TE ((1000000+(DALI_BAUD))/(2*(DALI_BAUD)))  //417us
#define DALI_TE_MIN (80*DALI_TE)/100
#define DALI_TE_MAX (120*DALI_TE)/100
#define DALI_IS_TE(x) ((DALI_TE_MIN)<=(x) && (x)<=(DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2*(DALI_TE_MIN))<=(x) && (x)<=(2*(DALI_TE_MAX)))


volatile byte varCompteur = 0; // La variable compteur_envoi
volatile byte ValeurLuminaire = 100; // La variable compteur_envoi


// Stocker le retour 
volatile Retour DaliR;




//###########################################################################
// Interruption pour transmission
//###########################################################################
hw_timer_t * transmittimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


static DALI *IsrTimerHook1;
static DALI *IsrTimerHook2;

//Rx Interrupt
static DALI *IsrPCINTHook;



//###########################################################################
// Transmit ISR
//###########################################################################
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);   
    //Jum only into interrupt when Dali defined
    //IsrTimerHook1->compteur_reception++;
    if(IsrTimerHook1==NULL) {return;}
    IsrTimerHook1->ISR_timer();
  portEXIT_CRITICAL_ISR(&timerMux);
  }


// This is the Transmit interrupt happening every 416us
void DALI::ISR_timer()
{
    portENTER_CRITICAL_ISR(&timerMux);   
    //this->compteur_envoi++;
    if(bus_idle_te_cnt<0xff) {
        bus_idle_te_cnt++;
        //this->compteur_reception++;
        //this->compteur_envoi++;
    }
    
    //send starbit, message bytes, 2 stop bits.
   switch(this->tx_state) {
        case IDLE:
        if ( (this->status_wait != RX_ANSWER) and (this->bus_idle_te_cnt >= 23) and (this->status_wait != RX_RECEIVING)) {
                this->status_wait=RX_NOANSWER;
            }
            break;
        case START:
            //wait for timeslot, then send start bit
            
            if(this->bus_idle_te_cnt >= 22) { // la plupart du temps déjà à 0xFF
                
                DALI_BUS_LOW();
                this->tx_state=START_X;
                this->status_wait=RX_NOANSWER;
            }
            break;
        case START_X:
            DALI_BUS_HIGH();
            this->tx_pos=0;
            this->tx_state=BIT;
            break;
        case BIT:
            if(this->tx_msg[this->tx_pos>>3] & 1<<(7-(this->tx_pos&0x7))) {DALI_BUS_LOW();} else {DALI_BUS_HIGH();}
            this->tx_state=BIT_X;
            break;
        case BIT_X:
            if(this->tx_msg[this->tx_pos>>3] & 1<<(7-(this->tx_pos&0x7))) {DALI_BUS_HIGH();} else {DALI_BUS_LOW();}
            this->tx_pos++;
            if(this->tx_pos < this->tx_len) {this->tx_state=BIT;} else {this->tx_state=STOP1;}
            break;
        case STOP1:
            DALI_BUS_HIGH();
            this->tx_state=STOP1_X;
            break;
        case STOP1_X:
            this->tx_state=STOP2;
            
            break;
        case STOP2:
            this->tx_state=STOP2_X;
            break;
        case STOP2_X:
            this->tx_state=STOP3;
            
            break;
        case STOP3:
            //this->receive_end = micros() + 9170;
            this->compteur_envoi++;
            this->status_wait=RX_WAITING;
            this->bus_idle_te_cnt=0;
            this->tx_state = IDLE;
            this->rx_state = RX_IDLE;
            this->rx_reaction = false;
            break;
    }
    

//    if(this->rx_state == RX_START) {
//        if(this->EventHandlerReceivedData!=NULL) this->EventHandlerReceivedData(this, (uint8_t*)this->rx_msg, 0);
//        }
    
    //Handle the delay of receiving
    
  
    
    //handle receiver stop bits
    if(this->rx_state == RX_BIT && this->bus_idle_te_cnt>4) {
        this->rx_state = RX_IDLE;
        this->status_wait=RX_ANSWER;
        //received two stop bits, got message in rx_msg + rx_len
        uint8_t bitlen = (this->rx_len+1)>>1;
        if((bitlen & 0x7) == 0) {
            uint8_t len=bitlen>>3;
            this->compteur_reception++;
            if(this->EventHandlerReceivedData!=NULL) this->EventHandlerReceivedData(this, (uint8_t*)this->rx_msg, len);
        }else{
            //invalid bitlen
            //TODO handle this
        }  
    }
  portEXIT_CRITICAL_ISR(&timerMux);
}

//###########################################################################
// Receiver ISR
//###########################################################################
void IRAM_ATTR ISRpinchange() {
    IsrPCINTHook->ISR_pinchange();
}


void DALI::ISR_pinchange() {   // Appele uniquement si changement de valeur
    uint32_t ts = micros(); //get timestamp of change
    
    this->compteur_reception++;
    this->bus_idle_te_cnt=0; //reset idle counter
    uint8_t bus_low = DALI_IS_BUS_LOW();
    uint8_t bus_high = DALI_IS_BUS_HIGH();
    DaliR.Resulat[DaliR.Position] = (DaliR.Resulat[DaliR.Position] << 1) | bus_high;
    DaliR.bitPos++;
    if (DaliR.bitPos==32)
     {
            DaliR.bitPos=0;
            DaliR.Position++;
            DaliR.Resulat[DaliR.Position]=0;
     }
        
    //exit if transmitting
    if(this->tx_state!=IDLE) {
        //this->status_wait = RX_ANSWER;
        return;
    /*
        //check tx collision
        if(bus_low && !this->tx_bus_low) {
            this->tx_state=IDLE; //stop transmitter
            this->tx_collision=1; //mark collision
        }
        return;
    */
     }
    this->rx_reaction = true;
    //Attention : Lecture inversée du protocole DALI :
    // Si DALI LOW ==> Le pin passe en HAUT
    
    //this->compteur_reception++;
    //no bus change, ignore  ****************
    if(bus_low == this->rx_last_bus_low) return;
    //this->rx_state = RX_RETOUR;
    
    
    //store values for next loop
    uint32_t dt = ts - this->rx_last_change_ts;  //Depuis combien de temps le bit est a ce niveau ?
    this->rx_last_change_ts = ts;
    this->rx_last_bus_low = bus_low;
    
    switch(this->rx_state) {
        case RX_IDLE:
            if(bus_high) {
                this->rx_state = RX_START;
                this->status_wait = RX_RECEIVING;
                //this->status_wait = RX_ANSWER;
            }
            break;
        case RX_START:    // 1 logique
            if(bus_high || !DALI_IS_TE(dt)) {// Bus haut ou Mauvaise syncho
                this->rx_state = RX_IDLE;
            }else{      // On est repassé au niveau bas... on initialise
                this->rx_len=-1;
                for(uint8_t i=0;i<7;i++) this->rx_msg[0]=0;  //
                this->rx_state = RX_BIT;
                
            }
            break;
        case RX_BIT:
            if(DALI_IS_TE(dt)) {
                //got a single Te pulse
                this->push_halfbit(bus_high);
            } else if(DALI_IS_2TE(dt)) {
                //got a double Te pulse
                this->push_halfbit(bus_high);
                this->push_halfbit(bus_high);
            } else {
                //got something else -> no good
                this->rx_state = RX_IDLE;
                this->status_wait = RX_NOANSWER;
                //TODO rx error
                return;
            }		
            break;
    }


    
}

void DALI::push_halfbit(uint8_t bit) {
    bit = (~bit)&1;
    if((this->rx_len & 1)==0) {
        uint8_t i = this->rx_len>>4;
        if(i<3) {
            this->rx_msg[i] = (this->rx_msg[i]<<1) | bit;
        }
    }
    this->rx_len++;
}






void DALI::begin(int8_t tx_pin,int8_t rx_pin)
{
    this->tx_pin=tx_pin;
    this->rx_pin=rx_pin;
    this->tx_state = IDLE;
    this->rx_state = RX_IDLE;
    this->compteur_envoi = 0;
    this->compteur_reception = 0;

    bus_idle_te_cnt=0;
    tx_msg[0]=0xFF;
    tx_msg[1]=B00000000;
    tx_len=16;


    DaliR.bitPos=0;
    DaliR.Position=0;
    DaliR.Resulat[0]=0;


    //setup tx
    if(this->tx_pin>=0) {
        //setup tx pin
        pinMode(this->tx_pin, OUTPUT);
        DALI_BUS_HIGH();


        // Dali protocol :
        // https://www.mouser.fr/applications/lighting-digitally-addressable/
        // 1 bit est codé sur 833 us  (C'est la variation),  le signal montant ou descendant lui est 416us

        //timer speed (Hz) = Timer clock speed (Mhz) / prescaler

        transmittimer = timerBegin(0, 8, true);   // fréquence pour 80Mzh donc incremtation touts les 1/ 1 000 000 secondes
        timerAttachInterrupt(transmittimer, &onTimer, true);
        // On veut appeler toutes les 416us
          timerAlarmWrite(transmittimer, 4163, true);
        //timerAlarmWrite(transmittimer, 416000, true);
        

        //setup timer interrupt hooks
        IsrTimerHook1 = this;
        timerAlarmEnable(transmittimer);
    }

    //setup rx
    this->rx_last_bus_low = 1;
    this->rx_last_change_ts = 0;

    if(this->rx_pin>=0) {
        //setup rx pin
        pinMode(this->rx_pin, INPUT);
        IsrPCINTHook = this;
        attachInterrupt(this->rx_pin, ISRpinchange, CHANGE);    
    }
    byte Adresse_lum;

    Nb_luminaires = 2;   // ********************************************


    // Lire les valeurs Min des luminaires
    for (int8_t i = 1; i<=Nb_luminaires; i++) {
        Adresse_lum = i << 1 | 1;
        Serial.println("Zero envoi");
        Serial.print(this->compteur_envoi);
        Serial.print(" ");
        Serial.println(this->compteur_reception);
        Commande_speciale(Adresse_lum,0xA2);
//        delay(100);
//         delay(1000);
        Luminaires[i].Min_level=rx_msg[0];
        Luminaires[i].Puissance_vari=rx_msg[0];
    }

        Serial.println("Apres la premiere commande spéciale");
        Serial.print(this->compteur_envoi);
        Serial.print(" ");
        Serial.println(this->compteur_reception);



}


void DALI::printResultat(){
    Serial.print("Reception ");
    for (size_t i = 0; i < DaliR.Position; i++)
    {    
        Serial.print(DaliR.Resulat[i]);
        Serial.print(" ");
    }
    Serial.println("Fin");


}



void DALI::down(uint8_t adresse)
{
    byte Adresse_lum = adresse << 1 & 0x7E | 0x01;
    tx_msg[0]=Adresse_lum;
    tx_msg[1]=4;
    tx_state=START;
    Luminaires[adresse].Puissance--;
    delay(25);
}



void DALI::query_level(uint8_t adresse)
{
    byte Adresse_lum = (adresse << 1 )| 1;
    tx_msg[0]=Adresse_lum;
    tx_msg[1]=0xA0;  //Query actual level
    tx_state=START;
    delay(25);
}

//Commande_speciale(0x03,0xA1);

void DALI::Commande(uint8_t Command)
{
    tx_msg[0]=0xFF;
    tx_msg[1]=Command;  //Query actual level
    tx_state=START;
    delay(25);
}


void DALI::Commande_speciale(uint8_t c1, uint8_t c2)

{
    tx_msg[0]=c1;
    tx_msg[1]=c2;
    tx_state=START;
    delay(26);
}





