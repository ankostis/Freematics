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
  Buzzer(const uint8_t pin = 0, const uint8_t duty_bits = 8,
         const uint8_t channel = 0) {
    setPin(pin, duty_bits, channel);
  }
  void setPin(const uint8_t pin, const uint8_t duty_bits = 8,
              const uint8_t channel = 0) {
    this->pin = pin;
    this->channel = channel;
    this->duty_bits = duty_bits;
    if (pin > 0) {
      ledcSetup(channel, 2000, duty_bits);
      ledcAttachPin(pin, channel);
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
  void tone(const float freqHz, const float volume = 1.0) {
    if (pin) {
      if (freqHz && volume) {
        ledcWriteTone(channel, freqHz);
        const int duty_cycle = volume * (1 << (duty_bits + 1));
        ledcWrite(channel, duty_cycle);
      } else {
        ledcWrite(channel, 0);
      }
    } else {
      ESP_LOGW(TAG, "Buzzer.tone(%f, %f) muted", freqHz, volume);
    }
  }
  // protected:  // No protection, adults may inspect them.
  uint8_t pin = 0;
  uint8_t duty_bits = 0;
  uint8_t channel = 0;
};
