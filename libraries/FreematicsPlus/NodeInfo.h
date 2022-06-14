/******************************************************************************
 * Freematics Hub client and Traccar client implementations
 * Works with Freematics ONE+
 * Developed by Stanley Huang <stanley@freematics.com.au>
 * Distributed under BSD license
 * Visit https://freematics.com/products for hardware information
 * Visit https://hub.freematics.com to view live and history telemetry data
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

#pragma once

struct freematics_cfg_t {
  char devid[12]; // populated after boot
  char vin[18]; // populated after boot

  /** NOTE: remember to increase `enable_flags` size if more flags added. */
  uint16_t enable_flags;
  int serial_autoconf_timeout;
  uint8_t log_level_run;
  uint8_t log_level_build;
  uint8_t log_sink;
  const char* log_sink_fpath;
  float log_sink_disk_usage_purge_prcnt;
  int nslots;
  int slot_len;
  int serialize_len;
  /** Bit for enable/disable decided on compile-time.  */
  uint8_t storage;
  /** NOTE: changes here, must convey to platformIO's monitor-filter. */
  uint8_t gnss;
  const char* ota_update_url;
  const char* ota_update_cert_pem;
  uint8_t net_dev;
  const char* wifi_ssd;
  const char* wifi_pwd;
  const char* cell_apn;
  const char* sim_card_pin;
  uint8_t srv_proto;
  const char* srv_host;
  const char* srv_path;
  uint16_t srv_port;
  uint32_t net_recv_timeout;
  uint32_t srv_sync_timeout;
  uint8_t net_retries;
  uint16_t net_udp_reconnect_delay;
  uint16_t stationary_timeout_vals[3];
  uint16_t data_interval_vals[3];
  uint32_t obfcm_interval;
  uint8_t obd_max_errors;
  uint16_t ping_back_interval;
  uint8_t wakeup_reset;
  float wakeup_motion_thr;
  float wakeup_jumpstart_thr;
  float cool_temp;
  uint16_t cool_delay;
  uint8_t pin_sensor1;
  uint8_t pin_sensor2;
};

/**
 * Sample output:
 *
 * ```
 * device_id: ABCDFGZRJ
 * +--     board: ESP32-D0WDQ6-v1@160x2,
 * +--       mac: f4f78bc40a24,
 * +--     flash: 16MiB@40MHz,
 * +--  slow_rtc: 0@150KHz,
 * +-- esp32_sdk: v4.4-367-gc29343eb94,
 * +--   arduino: 2.0.3,
 * +--fw_version: v0.1.0-upstream202204-27-g5b707fd,
 * +--sketch_elf: 183ec6f-3f4000b0,
 * +--     build: Apr 13 2022 13:37:40 (user@host),
 * +--last_boot: 1,
 * +-- partition:  20.03%, sketch_size:  262528, partition_size: 1310720,
 * +--      heap:   6.84%,   heap_used:   25248,      heap_size:  369160
 * ```
 *
 * ATTENTION: rev1 devices (like the sample above),
 * [need the PSRAM workaround](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html#esp32-rev-v1)
 *
 */
void log_node_info(const freematics_cfg_t &node_cfg);
