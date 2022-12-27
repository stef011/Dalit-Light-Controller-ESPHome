#ifndef MANCHESTER_BUS_H
#define MANCHESTER_BUS_H

#include <inttypes.h>
#include <Arduino.h>

/*---------------------------------------------------------------------------\
| Helper Macros                                                             |
|                                                                           |
/*---------------------------------------------------------------------------*/

#define DALI_BUS_LOW()             \
  digitalWrite(this->tx_pin, LOW); \
  this->tx_bus_low = 1
#define DALI_BUS_HIGH()             \
  digitalWrite(this->tx_pin, HIGH); \
  this->tx_bus_low = 0
#define DALI_BAUD 1200
#define DALI_TE ((1000000) / (2 * (DALI_BAUD))) // 417us
#define DALI_TE_MIN (80 * DALI_TE) / 100
#define DALI_TE_MAX (120 * DALI_TE) / 100
#define DALI_IS_TE(x) ((DALI_TE_MIN) <= (x) && (x) <= (DALI_TE_MAX))
#define DALI_IS_2TE(x) ((2 * (DALI_TE_MIN)) <= (x) && (x) <= (2 * (DALI_TE_MAX)))

#define DALI_IS_BUS_LOW() (digitalRead(this->rx_pin) == LOW)
#define DALI_IS_BUS_HIGH() (digitalRead(this->rx_pin) == HIGH)

/*---------------------------------------------------------------------------\
| ManchesterDaliBus                                                          |
|                                                                           |
| This class implements a Manchester encoded DALI bus.                      |
|                                                                           |
| The bus is implemented as a state machine. The state machine is           |
| implemented in the ISR_timer() function.                                  |
|                                                                           |
/*---------------------------------------------------------------------------*/

class ManchesterDaliBus
{
public:
  uint8_t begin(uint8_t tx_pin, uint8_t rx_pin);
  uint8_t sendFrame(uint8_t address, uint8_t msg);
  void IRAM_ATTR ISR_timer();
  void ISR_pinChange();

private:
  enum tx_statusEnum
  {
    IDLE,
    START,
    START_X,
    ADDR_BIT,
    ADDR_BIT_X,
    BIT,
    BIT_X,
    STOP1,
    STOP1_X,
    STOP2,
    STOP2_X,
    STOP3
  };
  uint8_t tx_pin;
  uint8_t tx_msg;
  uint8_t tx_msg_pos;
  uint8_t tx_addr;
  uint8_t tx_addr_pos;
  bool tx_bus_low;
  tx_statusEnum tx_state = IDLE;

  enum rx_statusEnum
  {
    RX_IDLE,
    RX_START,
    RX_BIT
  };
  rx_statusEnum rx_state;
  uint8_t rx_pin;
  uint8_t rx_last_state;
  uint32_t rx_last_change;
  uint8_t rx_msg;
  uint8_t rx_len;
  uint8_t rx_msg_pos;
  uint8_t message_received;

  void push_halfbit(uint8_t bit);

  volatile uint8_t bus_idle_te_cnt = 0;
};

#endif
