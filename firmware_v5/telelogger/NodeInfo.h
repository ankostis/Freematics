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
  /**
   * Node infos is a JSON string as output
   * (optionally with sensitive infos hidden) like this:
   *
   * ```JSON
   * {
   *   "device_id": "A0HNZRJU",
   *   "vin": "",
   *   "app_desc": "jrcmatic(ankostis@kudos):jrc-v0.0.3-109-g6ac3f4a-dirty",
   *   "build_date": "13 Jul 2022, 18:57:01+0300",
   *   "node_hw": {
   *     "board": "ESP32-D0WDQ6-v1",
   *     "cpu": "2x160MHz",
   *     "mac": "240ac48bf7f4",
   *     "flash": "4MiB@40MHz",
   *     "slow_rtc": "0@150KHz"
   *   },
   *   "node_fw": {
   *     "macroflags": (6f3 --> 0x6f3: ENABLE_OBD|ENABLE_MEMS|ENABLE_BUZZING_INIT|ENABLE_OTA_UPDATE|_NEED_SD|_NEED_SPIFFS|USE_ESP_IDF_LOG|HIDE_SECRETS_IN_LOGS),
   *     "reconf": 3,
   *     "reconf_fpath": "config.json",
   *     "arduino_ver": "2.0.4",
   *     "esp32_ver": "v4.4.1-472-gc9140caf8c",
   *     "ota_parts_used": "x2, R0, B0, U1",
   *     "partitions": [
   *       {
   *         "part": "app0:0x10000:160000",
   *         "ota_state": "0xffffffff",
   *         "app_desc": "jrcmatic(ankostis@kudos):jrc-v0.0.3-109-g6ac3f4a-dirty",
   *         "build_date": "13 Jul 2022, 18:57:01+0300",
   *         "part_sha256": "002b0365"
   *       },
   *       {
   *         "part": "app1:0x170000:160000",
   *         "ota_state": "0xffffffff",
   *         "app_desc": "jrcmatic(ankostis@kudos):jrc-v0.2.0-10-gc9bea54-dirty",
   *         "build_date": "12 Jul 2022, 21:01:37+0300",
   *         "part_sha256": "ac02da04"
   *       }
   *     ],
   *     "log_level_build": 3,
   *     "nslots": 256,
   *     "slot_len": 180,
   *     "storage": 0,
   *     "gnss": 1,
   *     "srv_proto": 1,
   *     "stationary_timeout_vals": [
   *       20,
   *       40,
   *       60
   *     ],
   *     "data_interval_vals": [
   *       1000,
   *       2000,
   *       5000
   *     ]
   *   },
   *   "node_state": {
   *     "last_boot_reason": 1,
   *     "partition_size": 1441792,
   *     "sketch_size": 1353584,
   *     "partition_max_use": 93.8820571899414,
   *     "boot_heap_size": 151728,
   *     "heap_size": 300980,
   *     "heap_max_used": 160432,
   *     "heap_max_use": 53.30320951558243,
   *     "heap_free_min": 140548,
   *     "esp_get_free_heap_size": 139980,
   *     "esp_get_free_internal_heap_size": 139816,
   *     "ESP_getFreeHeap": 183464,
   *     "ESP_getMinFreeHeap": 183224,
   *     "ESP_getMaxAllocHeap": 110580,
   *     "def_total_allocated": 116040,
   *     "def_total_free": 141176,
   *     "def_minimum_free": 140548,
   *     "def_largest_free_block": 110580
   *   },
   *   "cfg": {
   *     "obd_pipe_sec": 7,
   *     "log_levels": {
   *       "*": 3
   *     },
   *     "log_sink": 1,
   *     "log_sink_fpath": "/foo.txt",
   *     "log_sink_disk_usage_purge_prcnt": 0.8999999761581421,
   *     "log_sink_sync_interval_ms": 3141,
   *     "serialize_len": 1024,
   *     "ota_url": "<len: 37>",
   *     "ota_update_cert_pem_len": 3749,
   *     "net_dev": 1,
   *     "wifi_ssd": "GreatWifi",
   *     "wifi_pwd": "***",
   *     "cell_apn": "<len: 19>",
   *     "sim_card_pin": "***",
   *     "srv_host": "<len: 22>",
   *     "srv_path": "***",
   *     "srv_port": 0,
   *     "net_recv_timeout_ms": 5000,
   *     "srv_sync_timeout_ms": 120000,
   *     "net_retries": 5,
   *     "net_udp_reconnect_delay_ms": 3000,
   *     "obfcm_interval": 120000,
   *     "obd_max_errors": 3,
   *     "ping_back_interval_sec": 900,
   *     "reboot_on_wakeup": 0,
   *     "wakeup_motion_thr": 0.4000000059604645,
   *     "wakeup_jumpstart_thr": 13.600000381469727,
   *     "cool_temp": 80.0,
   *     "cool_delay_sec": 5,
   *     "pin_sensor1": 34,
   *     "pin_sensor2": 26,
   *     "bar": 3,
   *     "/spiffs/config.json": "error: [json.exception.parse_error.101] parse error at line 1, column 1: syntax error while parsing value - unexpected end of input; expected '[', '{', or a literal"
   *   },
   *   "cfg_build": {
   *     "obd_pipe_sec": 7,
   *     "log_levels": {
   *       "*": 3
   *     },
   *     "log_sink": 1,
   *     "log_sink_fpath": "/logs.txt",
   *     "log_sink_disk_usage_purge_prcnt": 0.8999999761581421,
   *     "log_sink_sync_interval_ms": 3141,
   *     "serialize_len": 1024,
   *     "ota_url": "<len: 37>",
   *     "ota_update_cert_pem_len": 3749,
   *     "net_dev": 1,
   *     "wifi_ssd": "GreatWifi",
   *     "wifi_pwd": "***",
   *     "cell_apn": "<len: 19>",
   *     "sim_card_pin": "***",
   *     "srv_host": "<len: 22>",
   *     "srv_path": "***",
   *     "srv_port": 0,
   *     "net_recv_timeout_ms": 5000,
   *     "srv_sync_timeout_ms": 120000,
   *     "net_retries": 5,
   *     "net_udp_reconnect_delay_ms": 3000,
   *     "obfcm_interval": 120000,
   *     "obd_max_errors": 3,
   *     "ping_back_interval_sec": 900,
   *     "reboot_on_wakeup": 0,
   *     "wakeup_motion_thr": 0.4000000059604645,
   *     "wakeup_jumpstart_thr": 13.600000381469727,
   *     "cool_temp": 80.0,
   *     "cool_delay_sec": 5,
   *     "pin_sensor1": 34,
   *     "pin_sensor2": 26
   *   },
   *   "cfg_sd": {
   *     "log_sink_fpath": "/foo.txt",
   *     "bar": 3
   *   },
   *   "cfg_fl": {
   *     "/spiffs/config.json": "error: [json.exception.parse_error.101] parse error at line 1, column 1: syntax error while parsing value - unexpected end of input; expected '[', '{', or a literal"
   *   }
   * }
   *
   * ```
   */

#pragma once

#include "config.h"
#include <hexbin.h>

#include <Esp.h>
#include <esp_app_format.h>
#include <esp_image_format.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_heap_caps.h>

#include <soc/rtc.h>

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <json.hpp>

#if BOARD_HAS_PSRAM && BOARD_HAS_PSRAM_HIGH
#include "esp32/himem.h"
#endif

typedef nlohmann::ordered_json Json;
typedef std::map<std::string, esp_log_level_t> LogLevels;
typedef uint16_t macroflags_t;
struct PartRec {
  const esp_partition_t * part;
  const esp_app_desc_t desc;
  const esp_ota_img_states_t state;
};
typedef std::vector<PartRec> PartInfos;


Json read_json(const char *fname);

/**
 * Idempotent transformation of freematics MACs --> device-ids,
 * like: 0xF4F78BC40A24 --> "A0HNZRJU".
 *
 * :param mac:
 *    the mac-address, from `ESP.getEfuseMac()`
 * :return:
 *    a x12-char  string (to be stored on :field:`node_info_t.device_id`)
 */
std::string mac_to_device_id(uint64_t max);

Json read_n_merge_cfg_sources(uint reconf, const char *reconf_fpath);

struct node_info_t {
  Json hw_info_to_json() const;
  Json fw_info_to_json(const PartInfos precs) const;
  Json node_state_to_json() const;
  Json config_to_json() const;

  /**
   * Produces a valid JSON string as output, like the pone on file header.
   */

  Json to_json() const;

  //////////
  // HARDWARE
  const uint64_t mac{ESP.getEfuseMac()};
  const std::string device_id{mac_to_device_id(mac)};

  //////////
  // FIRMWARE
  /**
   * ATTENTION: increase `macroflags` size if more macroflags added.
   * NOTE: remember to teach platformIO's monitor-filter about new macroflags.
   */
  const macroflags_t macroflags{
    ((ENABLE_OBD && 1) << 0)
    | ((ENABLE_MEMS && 1) << 1)
    | ((ENABLE_ORIENTATION && 1) << 2)
    | ((ENABLE_OLED && 1) << 3)
    | ((ENABLE_BUZZING_INIT && 1) << 4)
    | ((ENABLE_OTA_UPDATE && 1) << 5)
    | ((_NEED_SD && 1) << 6)
    | ((_NEED_SPIFFS && 1) << 7)
    | ((ENABLE_MULTILOG && 1) << 8)
    | ((USE_ESP_IDF_LOG && 1) << 9)
    | ((HIDE_SECRETS_IN_LOGS && 1) << 10)
    | ((BOARD_HAS_PSRAM && 1) << 11)
    | ((BOARD_HAS_PSRAM_HIGH && 1) << 12)
  };
  const uint8_t log_level_build{CORE_DEBUG_LEVEL};
  const uint reconf{RECONF};
  const char *reconf_fpath{RECONF_FPATH};
  const int nslots{BUFFER_SLOTS};
  const int serialize_len{SERIALIZE_BUFFER_SIZE};
  const int slot_len{BUFFER_LENGTH};
  const uint16_t stationary_timeout_vals[3]{STATIONARY_TIME_TABLE};
  const uint16_t data_interval_vals[3]{DATA_INTERVAL_TABLE};
  /** Bit for enable/disable decided GLOBALon compile-time.  */
  const uint8_t storage{STORAGE};
  const uint8_t gnss{GNSS};
  const uint8_t srv_proto{SERVER_PROTOCOL};
  const uint32_t partition_size{esp_ota_get_running_partition()->size};
  const uint32_t sketch_size{ESP.getSketchSize()};
  const float partition_use{(float) 100.0 * sketch_size / partition_size};
  const uint32_t boot_heap_size{ESP.getHeapSize()};

  //////////
  // STATUS
  char vin[18]{};
  int last_boot{};
  uint32_t heap_used{};
  float heap_use{};
#if BOARD_HAS_PSRAM
  uint32_t psram_used{};
  float psram_use{};
#if BOARD_HAS_PSRAM_HIGH
  uint32_t psramh_used{};
  float psramh_use{};
#endif
#endif

  //////////
  // CONFIG
  int obd_pipe_sec{BOOT_OBD_PIPE_TIMEOUT_SEC};
  LogLevels log_levels{RUNTIME_LOG_LEVELS};
  uint8_t log_sink{LOG_SINK};  // TODO: reconfigureable
  const char *log_sink_fpath{LOG_SINK_FPATH};
  float log_sink_disk_usage_purge_prcnt{LOG_SINK_DISK_USAGE_PURGE_RATIO};
  int32_t log_sink_sync_interval_ms{LOG_SINK_SYNC_INTERVAL_MS};
  const char *ota_url{OTA_UPDATE_URL};
  const char *ota_update_cert_pem{OTA_UPDATE_CERT_PEM};
  uint8_t net_dev{NET_DEVICE};  // TODO: reconfigurable
  const char *wifi_ssd{WIFI_SSID};
  const char *wifi_pwd{WIFI_PASSWORD};
  const char *cell_apn{CELL_APN};
  const char *sim_card_pin{SIM_CARD_PIN};
  const char *srv_host{SERVER_HOST};
  const char *srv_path{SERVER_PATH};
  uint16_t srv_port{SERVER_PORT};
  uint32_t net_recv_timeout_ms{DATA_RECEIVING_TIMEOUT_MS};
  uint32_t srv_sync_timeout_ms{SERVER_SYNC_INTERVAL_SEC * 1000};
  uint8_t net_retries{NET_CONNECT_RETRIES};
  uint16_t net_udp_reconnect_delay_ms{UDP_CONNECT_RETRY_DELAY_MS};
  uint32_t obfcm_interval{OBFCM_INTERVAL_MS};
  uint8_t obd_max_errors{MAX_OBD_ERRORS};
  uint16_t ping_back_interval_sec{PING_BACK_INTERVAL_SEC};
  uint8_t reboot_on_wakeup{REBOOT_ON_WAKEUP};
  float wakeup_motion_thr{MOTION_THRESHOLD};
  float wakeup_jumpstart_thr{THR_VOLTAGE};
  float cool_temp{COOLING_DOWN_TEMP};
  uint16_t cool_delay_sec{COOLING_DOWN_SLEEP_SEC};
  uint8_t pin_sensor1{PIN_SENSOR1};
  uint8_t pin_sensor2{PIN_SENSOR2};
};

const PartInfos collect_ota_partition_records();
//////
// Private but non-static, to be testable.
//
Json _partition_record_to_json(const PartRec &prec);

// TODO: prog should allow to be re-told when to check PSRAM writable.
#if BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
void validate_psram_can_indeed_write();
#endif  // BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
