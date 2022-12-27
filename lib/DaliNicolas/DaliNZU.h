
#if defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include <Arduino.h>
#endif // end IDE

//https://onlinedocs.microchip.com/pr/GUID-0CDBB4BA-5972-4F58-98B2-3F0408F3E10B-en-US-1/index.html?GUID-BC288B52-EF00-476C-9FFA-5EFDEDBED689

#ifndef DALI_h

#define DALI_h


#include <inttypes.h>


struct  Luminaire {
    byte    Puissance;      // Puissance actuelle du luminaire
    byte    Puissance_vari;  // Puissance stockée du variateur
    byte    Min_level;      // Niveau minimum
};

struct Retour {
    byte    Position;
    byte    bitPos;
    unsigned long Resulat[250];
};

class DALI
{

public:
    typedef void (*EventHandlerReceivedDataFuncPtr)(DALI *sender, uint8_t *data, uint8_t len);
    EventHandlerReceivedDataFuncPtr EventHandlerReceivedData;

    uint8_t     Nb_luminaires;  //Nombres de luminaires sur le bus
    Luminaire   Luminaires[10];  // Lien entre le luminaire commandé de l'exterieur et mon adresse
    byte        Correspondance[9]; // Conrrespondance entre le Numerro du luminaire et de la commande
    boolean     variaplus = false;  // Commande variation en cours.
    boolean     variamoins = false;  // Commande variation en cours.
    boolean     varisensplus = true; 
    byte        curr_adresse;       //Adress du luminaire en variation

    volatile int compteur_envoi;
    volatile int compteur_reception;
    
    void begin(int8_t tx_pin,int8_t rx_pin);
    
    void up(uint8_t adresse);
    void down(uint8_t adresse);
    void off();

    ///
    /// @brief	Commande de puissance directe
    /// @param	adresse Adresse du luminaire 1-64
    /// @param	Puissance Puissance désirée 0-255
    ///
    void direct_power(byte adresse, byte Puissance=255);

    void query_level(uint8_t adresse);
    void Commande(uint8_t Command);
    
    ///
    /// @brief	Envoi la sequence c1c2 sur le bus dali
    /// @param	c1 Premier byte
    /// @param	c2 Deuxieme byte
    ///
    void Commande_speciale(uint8_t c1, uint8_t c2);
    
    void printResultat();

    void On(byte adresse);
    
    void parse_command(byte Commande);
    
    void varistop(byte adresse);
    
    void variation();
    
    ///
    /// @brief	Scan le bus dali afin d'assigner des adresse courtes aux luminaires
    ///         les adresses serons de 1 à 64
    ///         Renseigne la variable de la classse : Nb_luminaires
    void SetShortadress();
    
    ///
    /// @brief	Identification des luminaires
    ///         Allume chaque luminaire à tour de role en faisant clignoter le nombre de fois son Nr
    ///         On attend la lecture du numero de lumuniare sur le port serie
    ///         Ce numero est assigné au luminaire
    void Identification();



    
    
    void ISR_timer();  // a supprimer 
    //void IRQ_send();
    void ISR_pinchange();
    
    void ISR_timer2_blink();
    
    
    
 //   #define DALI_HOOK_COUNT 3
    
private:
    uint8_t tx_pin; //transmitter pin
    uint8_t rx_pin; //receiver pin
    volatile boolean tx_bus_low;
    enum tx_stateEnum { IDLE=0,START,START_X,BIT,BIT_X,STOP1,STOP1_X,STOP2,STOP2_X,STOP3};
    volatile tx_stateEnum tx_state; //current state
    volatile uint8_t tx_collision; //collistion occured
    volatile uint8_t tx_pos; //current bit transmit position
    volatile uint8_t bus_idle_te_cnt; //number of Te since start of idle bus
    volatile uint8_t tx_msg[2];
    volatile uint8_t tx_len; //number of bits to transmit
    volatile unsigned long receive_end;   //timestamp for delay receiving (end + 9.17 ms)
    
    enum rx_stateEnum { RX_IDLE,RX_START,RX_BIT, RX_RETOUR};
    enum rx_responseEnum { RX_WAITING,RX_ANSWER,RX_NOANSWER,RX_RECEIVING};
    volatile rx_stateEnum rx_state; //current state
    volatile rx_responseEnum status_wait ; // Current waiting state
    volatile boolean rx_reaction;
    volatile uint8_t rx_last_bus_low; //receiver as low at last pinchange
    volatile uint32_t rx_last_change_ts; //timestamp last pinchange
    volatile uint8_t rx_msg[3]; //message received
    volatile int8_t rx_len; //number of half bits received
    
    void push_halfbit(uint8_t bit);
    bool Wait_answer();
    uint8_t getval(uint8_t Reg_address);



};

#endif