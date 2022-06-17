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

#include <string>
#include <json.hpp>

struct node_info_t {
  char device_id[12];
  char vin[18];

  //////////
  // CONFIG
  /** NOTE: remember to increase `enable_flags` size if more flags added. */
  uint16_t enable_flags;
  int serial_autoconf_timeout;
  uint8_t log_level_run;
  uint8_t log_level_build;
  uint8_t log_sink;
  const char *log_sink_fpath;
  float log_sink_disk_usage_purge_prcnt;
  int32_t log_sink_sync_interval_ms;
  int nslots;
  int slot_len;
  int serialize_len;
  /** Bit for enable/disable decided on compile-time.  */
  uint8_t storage;
  /** NOTE: changes here, must convey to platformIO's monitor-filter. */
  uint8_t gnss;
  const char *ota_url;
  const char *ota_update_cert_pem;
  uint8_t net_dev;
  const char *wifi_ssd;
  const char *wifi_pwd;
  const char *cell_apn;
  const char *sim_card_pin;
  uint8_t srv_proto;
  const char *srv_host;
  const char *srv_path;
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
 * Idempotent transformation of freematics MACs --> device-ids,
 * like: 0xF4F78BC40A24 --> "A0HNZRJU".
 *
 * :param mac:
 *    the mac-address, from `ESP.getEfuseMac()`
 * :param device_id:
 *    a 12-char buffer (stored on :field:`node_info_t.device_id`)
 */
void mac_to_device_id(uint64_t max, char *device_id);

/**
 * Produces a valid JSON string as output, like this:
 *
 * ```
 * {
 *    "device_id": "A0HNZRJU",
 *       "board": "ESP32-D0WDQ6-v1@160x2",
 *         "mac": "0xF4F78BC40A24",
 *       "flash": "4MiB@40MHz",
 *    "slow_rtc": "0@150KHz",
 *   "esp32_sdk": "v4.4.1-1-gb8050b365e",
 *     "arduino": "2.0.3",
 *  "fw_version": "jrc-v0.0.3-63-g5b615bb",
 *  "sketch_elf": "ed59c3e-3f4000b0",
 *  "build_date": "Jun 21 2022 12:02:57",
 *    "build_by": "ankostis@kudos",
 *   "last_boot": 1,
 *     "partition_size": 1310720,
 *        "sketch_size": 944880,
 *      "partition_use": 72.09,
 * "ota_partition_size": 1310720,
 *        "heap": 333620,
 *   "heap_used": 81300,
 *    "heap_use": 24.37,
 *       "psram": 4192139,
 *  "psram_used": 0,
 *   "psram_use": 0.00
 * }
 * ```
 *
 * Build as a string, to exploit `sprintf()` when constructing HW-strings.
 *
 * ATTENTION: rev1 devices (like the sample above), need the PSRAM workaround
 * (`-mfix-esp32-psram-cache-issue`) if `-DBOARD_HAS_PSRAM` build-flag enabled.
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html#esp32-rev-v1
 */
std::string generate_node_infos();
nlohmann::json node_info_to_json(const node_info_t &infos);