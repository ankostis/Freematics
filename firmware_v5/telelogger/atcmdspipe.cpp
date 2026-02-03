#include "atcmdspipe.h"
#include "FreematicsPlus.h"
#include "NodeInfo.h"

#if BOOT_AT_PIPE_TIMEOUT_SEC

#include <esp_log.h>
#include "config.h"
#include "NodeInfo.h"
#include "FreematicsBase.h"

inline constexpr const char TAG_ATPIPE[] = "ATPIPE";

void power_cycle_modem() {
    pinMode(PIN_BEE_PWR, OUTPUT);
    digitalWrite(PIN_BEE_PWR, LOW);
    delay(200);
    digitalWrite(PIN_BEE_PWR, HIGH);
}

void AtPipe::banner(int pipe_timeout_sec) {
  Serial.printf(
    "AT-PIPE: Type any %s AT cmds for %s module for %isec:"
    "\n  - [CTRL+P] - (P)ower-toggle on selected module"
    "\n  - [CTRL+N] - (N)ext module"
    "\n  - [CTRL+D] - en(D) session immediately"
    "\n",
    at_type, module_name, pipe_timeout_sec);
}

void AtPipe::begin() {
  Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  Serial1.setHwFlowCtrlMode(HW_FLOWCTRL_DISABLE);
}

void AtPipe::end() {
  Serial1.end();
}

static AtPipe at_pipes[] = {
  AtPipe{
    LINK_UART_BAUDRATE,
    PIN_LINK_UART_RX,
    PIN_LINK_UART_TX,
    "ELM327",
    "OBD",
    nullptr
  },
  AtPipe{
    BEE_BAUDRATE,
    PIN_BEE_UART_RXD,
    PIN_BEE_UART_TXD,
    "SIMCOM",
    "MODEM",
    power_cycle_modem
  }
};

void enter_atpipe_loop(uint32_t timeout_ms) {
  constexpr const size_t n_pipes = sizeof_array(at_pipes);
  constexpr const char EOT          = 0x04;  // (CTRL+D) End Of Transmission
  constexpr const char CTRL_POWER   = 0x10;  // (CTRL+P)
  constexpr const char CTRL_NEXT    = 0x0E;  // (CTRL+N)

  int mod_i = 0;
  AtPipe *bp = &at_pipes[mod_i];
  bp->banner(timeout_ms);
  bp->begin();

  uint32_t last_traffic_ms, now_ms;
  bool stop, input_given;
  last_traffic_ms = now_ms = millis();
  stop= input_given=false;
  do {
    // Module --> USB
    //
    while (Serial1.available()) {
      Serial.write(Serial1.read());
      last_traffic_ms = now_ms;
    }

    // USB --> Module, scanning for keyboard shortcuts.
    //
    if (Serial.available()) {
      char usb_char = Serial.read();

      switch (usb_char) {
        case EOT:
          Serial.printf("--(( USER BREAK ))--\n"); stop = true;
          break;
        case CTRL_POWER:
          if (bp->power_cycle_func) {
            Serial.printf("--(( TOGGLE POWER %s ))--\n", bp->module_name);
            bp->power_cycle_func();
          }
          break;
        case CTRL_NEXT: {
          bp->end();
          const char *old_name = bp->module_name;
          bp = &at_pipes[++mod_i % n_pipes];
          Serial.printf("--(( CYCLE from %s --> %s ))--\n", old_name, bp->module_name);
          bp->banner(timeout_ms);
          bp->begin();
          break;
        }
        default:
          Serial1.write(usb_char);

          // Echo user-input back to serial port.
          // Coproc's `ATE1` cmd seems not that adept...
          Serial.write(usb_char);
          input_given = true;
      }

      last_traffic_ms = now_ms;
    }

    now_ms = millis();
  } while (!stop && (now_ms - last_traffic_ms) < timeout_ms);

  if (input_given) {
    Serial.printf("--(( OBD_PIPE did things...REBOOTING! ))--\n");
    esp_restart();
  }
  Serial1.end();
  ESP_LOGW(TAG_ATPIPE, "--(( OBD_PIPE did nothing, booting continues ))--");
}

#endif // BOOT_AT_PIPE_TIMEOUT_SEC
