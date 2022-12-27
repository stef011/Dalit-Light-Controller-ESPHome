#include "esphome.h"
#include <ManchesterDaliBus.h>

class SalonLight : public Component, public LightOutput
{
private:
  ManchesterDaliBus daliLight;
  uint8_t short_address = 0xFE;

public:
  void setup() override
  {
    // daliLight.begin(2);
  }
  void begin(uint8_t tx_pin, uint8_t rx_pin, uint8_t short_address = 0xFE)
  {
    daliLight.begin(tx_pin, rx_pin);
    if (short_address != 0xFE)
    {
      this->short_address = short_address;
    }
    // daliLight.short_Address_Setup(short_address);
  }
  LightTraits get_traits() override
  {
    auto traits = LightTraits();
    traits.set_supported_color_modes({ColorMode::BRIGHTNESS, ColorMode::ON_OFF});
    return traits;
  }
  void write_state(LightState *state) override
  {
    float brightness;
    state->set_gamma_correct(0);
    state->current_values_as_brightness(&brightness);
    // ESP_LOGCONFIG("SalonLight", "Setting brightness to %.2f", brightness);
    uint8_t msg = brightness * 255;
    daliLight.sendFrame(short_address, msg);
  }
};
