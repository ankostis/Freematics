/*************************************************************************
 * Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
 * Distributed under BSD license
 * Visit https://freematics.com for more information
 * (C)2017-2022 Developed by Stanley Huang <stanley@freematics.com.au>
 *************************************************************************/
#pragma once

#include <esp32-hal-ledc.h>
#include <esp32-hal-log.h>

class Buzzer {
 public:
  Buzzer(uint8_t pin = 0, uint8_t duty_bits = 8) { setPin(pin, duty_bits); }
  void setPin(uint8_t pin, uint8_t duty_bits = 8) {
    this->pin = pin;
    if (pin > 0) {
      this->duty_bits = duty_bits;
      ledcSetup(0, 2000, duty_bits);
      ledcAttachPin(pin, 0);
    }
  }
  /**
   * volume:
   *      ∈ (0,2) of duty-cycle, so that 1 is the lowest (50% duty)
   *      and 0/1 approach the left/right edges; in essence
   *      NOTE: function as a volume-percent on high frequencies,
   *      and makes a double-click on low < 10Hz.
   *      Square it on high frequencies (ie > 30 Hz) to effectively set RMS.
   *      *
   * Setting any one of `freqHz`, `volume` to 0, mutes the buzzer.
   */
  void tone(float freqHz, float volume = 1.0) {
    if (pin) {
      if (freqHz && volume) {
        ledcWriteTone(0, freqHz);
        int duty_cycle = volume * volume * (1 << (duty_bits + 1));
        ledcWrite(0, duty_cycle);
      } else {
        ledcWrite(0, 0);
      }
    } else {
      ESP_LOGW(TAG, "Buzzer.tone(%f, %f) disabled - set a positive pin.",
               freqHz, volume);
    }
  }
  // protected:  // no protection, adults may inspectit on runtime.
  uint8_t pin = 0;
  uint8_t duty_bits = 0;
};
