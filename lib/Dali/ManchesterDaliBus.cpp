#include <ManchesterDaliBus.h>

hw_timer_s *tx_timer = NULL;
ManchesterDaliBus *bus;

// ###########################################################################
//  Transmitter ISR
// ###########################################################################

void IRAM_ATTR Tx_ISR()
{
  if (bus == NULL)
  {
    Serial.println("Bus is NULL");
    return;
  }

  bus->ISR_timer();
}

void IRAM_ATTR ManchesterDaliBus::ISR_timer()
{
  if (this->bus_idle_te_cnt < 0xFF)
  {
    this->bus_idle_te_cnt++;
  }

  switch (this->tx_state)
  {

  case IDLE:
    if (this->message_received)
    {
      Serial.print("Message received: ");
      Serial.println(this->rx_msg, HEX);
      this->message_received = false;
    }

    break;
  case START:
    if (this->bus_idle_te_cnt >= 22)
    {
      // Serial.print("Start: 0");
      DALI_BUS_LOW();
      this->tx_state = START_X;
    }
    break;
  case START_X:
    // Serial.print("1");
    DALI_BUS_HIGH();
    this->tx_msg_pos = 0;
    this->tx_addr_pos = 0;
    this->tx_state = ADDR_BIT;
    break;
  case ADDR_BIT:
    if (this->tx_addr >> (7 - (this->tx_addr_pos & 7)) & 1)
    {
      // Serial.print("0");
      DALI_BUS_LOW();
    }
    else
    {
      // Serial.print("1");
      DALI_BUS_HIGH();
    }
    this->tx_state = ADDR_BIT_X;
    break;
  case ADDR_BIT_X:
    if (this->tx_addr >> (7 - (this->tx_addr_pos & 7)) & 1)
    {
      // Serial.print("1");
      DALI_BUS_HIGH();
    }
    else
    {
      // Serial.print("0");
      DALI_BUS_LOW();
    }
    this->tx_addr_pos++;
    if (tx_addr_pos < 8)
    {
      this->tx_state = ADDR_BIT;
    }
    else
    {
      this->tx_state = BIT;
      // Serial.println("Data");
    }
    break;
  case BIT:
    if (this->tx_msg >> (7 - (this->tx_msg_pos & 7)) & 1)
    {
      // Serial.print("0");
      DALI_BUS_LOW();
    }
    else
    {
      // Serial.print("1");
      DALI_BUS_HIGH();
    }
    this->tx_state = BIT_X;
    break;
  case BIT_X:
    if (this->tx_msg >> (7 - (this->tx_msg_pos & 7)) & 1)
    {
      // Serial.print("1");
      DALI_BUS_HIGH();
    }
    else
    {
      // Serial.print("0");
      DALI_BUS_LOW();
    }
    this->tx_msg_pos++;
    if (this->tx_msg_pos < 8)
    {
      this->tx_state = BIT;
    }
    else
    {
      this->tx_state = STOP1;
    }
    break;
  case STOP1:
    // Serial.print("1");
    DALI_BUS_HIGH();
    this->tx_state = STOP1_X;
    break;
  case STOP1_X:
    // Serial.print("1");
    this->tx_state = STOP2;
    break;
  case STOP2:
    // Serial.print("1");
    this->tx_state = STOP2_X;
    break;
  case STOP2_X:
    // Serial.print("1");
    this->tx_state = STOP3;
    break;
  case STOP3:
    // Serial.println("1");
    this->bus_idle_te_cnt = 0;
    this->tx_state = IDLE;
    break;
  }
  // Serial.println(digitalRead(this->tx_pin));
}

// ###########################################################################
//  Receiver
// ###########################################################################

void Rx_ISR()
{
  if (bus == NULL)
  {
    return;
  }

  bus->ISR_pinChange();
}

void ManchesterDaliBus::ISR_pinChange()
{
  uint32_t now = micros();                   // Get the current time
  this->bus_idle_te_cnt = 0;                 // Reset the bus idle counter so we wait before sending again
  uint8_t state = digitalRead(this->rx_pin); // Get the current state of the pin

  // Avoid sending a message if the bus is not idle
  if (this->tx_state != IDLE)
  {
    return;
  }

  // No change, ignore
  if (this->rx_last_state == state)
  {
    return;
  }

  // Save values for the next call
  uint32_t dt = now - this->rx_last_change;
  this->rx_last_state = state;
  this->rx_last_change = now;

  switch (this->rx_state)
  {
  case RX_IDLE:
    if (this->rx_last_state == 0)
    {
      this->rx_state = RX_START;
    }
    break;
  case RX_START:
    if (this->rx_last_state == 1 && DALI_IS_TE(dt))
    {
      this->rx_state = RX_BIT;
      this->rx_msg_pos = 0;
    }
  case RX_BIT:
    if (DALI_IS_2TE(dt))
    {
      if (state == 0)
      {
        this->rx_msg |= 1 << (7 - (this->rx_msg_pos & 7));
      }
      else
      {
        this->rx_msg &= ~(1 << (7 - (this->rx_msg_pos & 7)));
      }
      this->rx_msg_pos++;
    }
    if (this->rx_msg_pos >= 8)
    {
      this->message_received = 1;
      this->rx_state = RX_IDLE;
    }
    break;
  }
}

/*--------------------------------------------*/

uint8_t ManchesterDaliBus::begin(uint8_t tx_pin, uint8_t rx_pin)
{
  this->tx_pin = tx_pin;
  this->rx_pin = rx_pin;

  pinMode(this->tx_pin, OUTPUT);

  if (bus == NULL)
  {
    bus = this;
  }
  else
  {
    return -1;
  }

  tx_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(tx_timer, &Tx_ISR, true);
  timerAlarmWrite(tx_timer, DALI_TE, true);
  timerAlarmEnable(tx_timer);

  attachInterrupt(digitalPinToInterrupt(this->rx_pin), Rx_ISR, CHANGE);
  // Serial.println(this->tx_pin);

  return 0;
}

/*--------------------------------------------
 *  Send a message to the DALI bus
 *  @param addr 8-bit address
 * @param msg 8-bit message
 * @return 0 on success, -1 if the bus is busy
 *--------------------------------------------*/
uint8_t ManchesterDaliBus::sendFrame(uint8_t address, uint8_t msg)
{
  if (this->tx_state != IDLE)
  {
    Serial.println("Bus busy");
    return -1;
  }

  this->tx_addr = address;
  this->tx_msg = msg;
  this->tx_state = START;
  return 0;
}
