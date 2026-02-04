/************************
 * Emulate terminal from USB to send AT commands to Freematics sub-modules (OBD & MODEM)
 */
#pragma once

#include <cstdint>

#if BOOT_AT_PIPE_TIMEOUT_SEC

/**
 * Power cycle the cellular modem module
 */
void power_cycle_modem();

/**
 * AT command pipe configuration structure
 */
struct AtPipe {
  unsigned long baud;
  int8_t rx_pin;
  int8_t tx_pin;
  const char *at_type;
  const char *module_name;
  void (*power_cycle_func)();

  void banner(int pipe_timeout_sec);
  void begin();
  void end();
};

/**
 * Pipe bidirectionally the USB directly into UART1, itself connected to some module
 * (USB <--> UART <--> (OBD | MODEM)) and reboot on exit if anything touched.
 *
 * @param timeout_ms: if negative, loops forever
 *
 * Keyboard shortcuts:
 *  - [CTRL+P] - (P)ower-toggle on selected module
 *  - [CTRL+N] - (N)ext module
 *  - [CTRL+D] - en(D) session immediately
 */
void enter_atpipe_loop(uint32_t timeout_ms);

#endif // BOOT_AT_PIPE_TIMEOUT_SEC
