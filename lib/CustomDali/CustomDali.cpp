#include <customDali.h>
#include <Arduino.h>

#define DALI_BUS_LOW()             \
  digitalWrite(this->tx_pin, LOW); \
  this->tx_bus_low = 1
#define DALI_BUS_HIGH()             \
  digitalWrite(this->tx_pin, HIGH); \
  this->tx_bus_low = 0
#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin) == LOW)
#define DALI_BAUD 1200
#define DALI_TE ((1000000 + (DALI_BAUD)) / (2 * (DALI_BAUD))) // 417us
#define DALI_TE_MIN (80 * DALI_TE) / 100
#define DALI_TE_MAX (120 * DALI_TE) / 100
#define DALI_IS_TE(x) ((DALI_TE_MIN) <= (x) && (x) <= (DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2 * (DALI_TE_MIN)) <= (x) && (x) <= (2 * (DALI_TE_MAX)))

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ###########################################################################
//  Transmitter ISR
// ###########################################################################
static Dali *IsrTimerHooks[DALI_HOOK_COUNT + 1];

// timer compare interrupt service routine
void IRAM_ATTR Tx_ISR()
{

  for (uint8_t i = 0; i < DALI_HOOK_COUNT; i++)
  {
    if (IsrTimerHooks[i] == NULL)
    {
      return;
    }
    IsrTimerHooks[i]->ISR_timer();
  }
}

void Dali::ISR_timer()
{
  if (this->bus_idle_te_cnt < 0xff)
    this->bus_idle_te_cnt++;

  // send starbit, message bytes, 2 stop bits.
  switch (this->tx_state)
  {
  case IDLE:
    break;
  case START:
    // wait for timeslot, then send start bit
    if (this->bus_idle_te_cnt >= 22)
    {
      DALI_BUS_LOW();
      this->tx_state = START_X;
    }
    break;
  case START_X:
    DALI_BUS_HIGH();
    this->tx_pos = 0;
    this->tx_state = BIT;
    break;
  case BIT:
    if (this->tx_msg[this->tx_pos >> 3] & 1 << (7 - (this->tx_pos & 0x7)))
    {
      DALI_BUS_LOW();
    }
    else
    {
      DALI_BUS_HIGH();
    }
    this->tx_state = BIT_X;
    break;
  case BIT_X:
    if (this->tx_msg[this->tx_pos >> 3] & 1 << (7 - (this->tx_pos & 0x7)))
    {
      DALI_BUS_HIGH();
    }
    else
    {
      DALI_BUS_LOW();
    }
    this->tx_pos++;
    if (this->tx_pos < this->tx_len)
    {
      this->tx_state = BIT;
    }
    else
    {
      this->tx_state = STOP1;
    }
    break;
  case STOP1:
    DALI_BUS_HIGH();
    this->tx_state = STOP1_X;
    break;
  case STOP1_X:
    this->tx_state = STOP2;
    break;
  case STOP2:
    this->tx_state = STOP2_X;
    break;
  case STOP2_X:
    this->tx_state = STOP3;
    break;
  case STOP3:
    this->bus_idle_te_cnt = 0;
    this->tx_state = IDLE;
    this->rx_state = RX_IDLE;
    break;
  }

  // handle receiver stop bits
  if (this->rx_state == RX_BIT && this->bus_idle_te_cnt > 4)
  {
    this->rx_state = RX_IDLE;
    // received two stop bits, got message in rx_msg + rx_len
    uint8_t bitlen = (this->rx_len + 1) >> 1;
    if ((bitlen & 0x7) == 0)
    {
      uint8_t len = bitlen >> 3;
      if (this->EventHandlerReceivedData != NULL)
        this->EventHandlerReceivedData(this, (uint8_t *)this->rx_msg, len);
    }
    else
    {
      // invalid bitlen
      // TODO handle this
    }
  }
}

// ###########################################################################
//  Receiver ISR
// ###########################################################################

static Dali *IsrHook;

void IRAM_ATTR
Rx_ISR()
{
  if (IsrHook != NULL)
  {
    IsrHook->ISR_pinchange();
  }
}

void Dali::ISR_pinchange()
{
  uint32_t ts = micros();    // get timestamp of change
  this->bus_idle_te_cnt = 0; // reset idle counter
  uint8_t bus_low = DALI_IS_BUS_LOW();

  // exit if transmitting
  if (this->tx_state != IDLE)
  {
    // check tx collision
    if (bus_low && !this->tx_bus_low)
    {
      this->tx_state = IDLE;  // stop transmitter
      this->tx_collision = 1; // mark collision
    }
    return;
  }

  // no bus change, ignore
  if (bus_low == this->rx_last_bus_low)
    return;

  // store values for next loop
  uint32_t dt = ts - this->rx_last_change_ts;
  this->rx_last_change_ts = ts;
  this->rx_last_bus_low = bus_low;

  switch (this->rx_state)
  {
  case RX_IDLE:
    if (bus_low)
    {
      this->rx_state = RX_START;
    }
    break;
  case RX_START:
    if (bus_low || !DALI_IS_TE(dt))
    {
      this->rx_state = RX_IDLE;
    }
    else
    {
      this->rx_len = -1;
      for (uint8_t i = 0; i < 7; i++)
        this->rx_msg[0] = 0;
      this->rx_state = RX_BIT;
    }
    break;
  case RX_BIT:
    if (DALI_IS_TE(dt))
    {
      // got a single Te pulse
      this->push_halfbit(bus_low);
    }
    else if (DALI_IS_2TE(dt))
    {
      // got a double Te pulse
      this->push_halfbit(bus_low);
      this->push_halfbit(bus_low);
    }
    else
    {
      // got something else -> no good
      this->rx_state = RX_IDLE;
      // TODO rx error
      return;
    }
    break;
  }
}

void Dali::push_halfbit(uint8_t bit)
{
  bit = (~bit) & 1;
  if ((this->rx_len & 1) == 0)
  {
    uint8_t i = this->rx_len >> 4;
    if (i < 3)
    {
      this->rx_msg[i] = (this->rx_msg[i] << 1) | bit;
    }
  }
  this->rx_len++;
}

// ###########################################################################
//  Dali Class
// ###########################################################################

void Dali::begin(int8_t tx_pin, int8_t rx_pin)
{
  this->tx_pin = tx_pin;
  this->rx_pin = rx_pin;
  this->tx_state = IDLE;
  this->rx_state = RX_IDLE;

  // setup tx
  if (this->tx_pin >= 0)
  {
    // setup tx pin
    pinMode(this->tx_pin, OUTPUT);
    DALI_BUS_HIGH();

    // // setup tx timer interrupt
    // TCCR1A = 0;
    // TCCR1B = 0;
    // TCNT1 = 0;

    // OCR1A = (F_CPU + (DALI_BAUD)) / (2 * (DALI_BAUD)); // compare match register 16MHz/256/2Hz
    // TCCR1B |= (1 << WGM12);                            // CTC mode
    // TCCR1B |= (1 << CS10);                             // 1:1 prescaler
    // TIMSK1 |= (1 << OCIE1A);                           // enable timer compare interrupt

    // setup tx timer interrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &Tx_ISR, true);
    timerAlarmWrite(timer, DALI_TE, true);
    timerAlarmEnable(timer);

    // setup timer interrupt hooks
    for (uint8_t i = 0; i < DALI_HOOK_COUNT; i++)
    {
      if (IsrTimerHooks[i] == NULL)
      {
        IsrTimerHooks[i] = this;
        break;
      }
    }
  }

  // setup rx
  if (this->rx_pin >= 0)
  {
    // setup rx pin
    pinMode(this->rx_pin, INPUT);

    // // setup rx pinchange interrupt
    // //  0- 7 PCINT2_vect PCINT16-23
    // //  8-13 PCINT0_vect PCINT0-5
    // // 14-19 PCINT1_vect PCINT8-13
    // if (this->rx_pin <= 7)
    // {
    //   PCICR |= (1 << PCIE2);
    //   PCMSK2 |= (1 << (this->rx_pin));
    //   IsrPCINT2Hook = this; // setup pinchange interrupt hook
    // }
    // else if (this->rx_pin <= 13)
    // {
    //   PCICR |= (1 << PCIE0);
    //   PCMSK0 |= (1 << (this->rx_pin - 8));
    //   IsrPCINT0Hook = this; // setup pinchange interrupt hook
    // }
    // else if (this->rx_pin <= 19)
    // {
    //   PCICR |= (1 << PCIE1);
    //   PCMSK1 |= (1 << (this->rx_pin - 14));
    //   IsrPCINT1Hook = this; // setup pinchange interrupt hook
    // }

    // setup rx pinchange interrupt

    IsrHook = this; // setup pinchange interrupt hook

    attachInterrupt(digitalPinToInterrupt(this->rx_pin), Rx_ISR, CHANGE);
  }
}

uint8_t Dali::send(uint8_t *tx_msg, uint8_t tx_len_bytes)
{
  if (tx_len_bytes > 3)
    return 2;
  if (this->tx_state != IDLE)
    return 1;
  for (uint8_t i = 0; i < tx_len_bytes; i++)
    this->tx_msg[i] = tx_msg[i];
  this->tx_len = tx_len_bytes << 3;
  this->tx_collision = 0;
  this->tx_state = START;
  return 0;
}

uint8_t Dali::sendwait(uint8_t *tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms)
{
  if (tx_len_bytes > 3)
    return 2;
  uint32_t ts = millis();
  // wait for idle
  while (this->tx_state != IDLE)
  {
    if (millis() - ts > timeout_ms)
      return 1;
  }
  // start transmit
  if (this->send(tx_msg, tx_len_bytes))
    return 2;
  // wait for completion
  while (this->tx_state != IDLE)
  {
    if (millis() - ts > timeout_ms)
      return 3;
  }
  // wait for answer
  // TODO
  return 0;
}

uint8_t Dali::sendwait_int(uint16_t tx_msg, uint32_t timeout_ms)
{
  uint8_t m[3];
  m[0] = tx_msg >> 8;
  m[1] = tx_msg & 0xff;
  return sendwait(m, 2, timeout_ms);
}

uint8_t Dali::sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms)
{
  uint8_t m[3];
  m[0] = tx_msg;
  return sendwait(m, 1, timeout_ms);
}
