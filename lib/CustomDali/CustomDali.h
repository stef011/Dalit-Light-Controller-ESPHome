#ifndef CUSTOM_DALI_H
#define CUSTOM_DALI_H
#include <inttypes.h>

class Dali
{
public:
  typedef void (*EventHandlerReceivedDataFuncPtr)(Dali *sender, uint8_t *data, uint8_t len);
  EventHandlerReceivedDataFuncPtr EventHandlerReceivedData;

  void begin(int8_t tx_pin, int8_t rx_pin);
  uint8_t send(uint8_t *tx_msg, uint8_t tx_len_bytes);
  uint8_t sendwait(uint8_t *tx_msg, uint8_t tx_len_bytes, uint32_t timeout_ms = 500);
  uint8_t sendwait_int(uint16_t tx_msg, uint32_t timeout_ms = 500);
  uint8_t sendwait_byte(uint8_t tx_msg, uint32_t timeout_ms = 500);
  void ISR_timer();
  void ISR_pinchange();

#define DALI_HOOK_COUNT 3

private:
  enum tx_stateEnum
  {
    IDLE = 0,
    START,
    START_X,
    BIT,
    BIT_X,
    STOP1,
    STOP1_X,
    STOP2,
    STOP2_X,
    STOP3
  };
  uint8_t tx_pin;
  uint8_t tx_msg[3];
  uint8_t tx_len;                 // number of bits to transmit
  volatile uint8_t tx_pos;        // current bit transmit position
  volatile tx_stateEnum tx_state; // current state
  volatile uint8_t tx_collision;  // collistion occured
  volatile uint8_t tx_bus_low;    // bus is low according to transmitter?

  enum rx_stateEnum
  {
    RX_IDLE,
    RX_START,
    RX_BIT
  };
  uint8_t rx_pin;                      // receiver pin
  volatile uint8_t rx_last_bus_low;    // receiver as low at last pinchange
  volatile uint32_t rx_last_change_ts; // timestamp last pinchange
  volatile rx_stateEnum rx_state;      // current state
  volatile uint8_t rx_msg[3];          // message received
  volatile int8_t rx_len;              // number of half bits received
  volatile uint8_t rx_last_halfbit;    // last halfbit received

  volatile uint8_t bus_idle_te_cnt; // number of Te since start of idle bus

  void push_halfbit(uint8_t bit);
};

#endif
