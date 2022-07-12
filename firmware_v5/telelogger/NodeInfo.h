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
#include <iomanip>
#include <json.hpp>
#include <map>
#include <sstream>
#include <vector>

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


struct node_info_t {
  Json hw_info_to_json() const;
  Json fw_info_to_json(const PartInfos precs) const;
  Json node_state_to_json() const;

  /**
   * Produces a valid JSON string as output, like this:
   *
   * ```
   * {
   *   "device_id": "A0HNZRJU",
   *   "vin": "",
   *   "app_desc": "jrcmatic(user@host):jrc-v0.0.3-83-gae610e9-dirty",
   *   "build_date": "06 Jul 2022, 15:22:57+0300",
   *   "node_hw": {
   *     "board": "ESP32-D0WDQ6-v1",
   *     "cpu": "2x160MHz",
   *     "mac": "240ac48bf7f40000",
   *     "flash": "4MiB@40MHz",
   *     "slow_rtc": "0@150KHz"
   *   },
   *   "node_fw": {
   *     "macroflags": (233 --> 0x233: ENABLE_OBD|ENABLE_MEMS|ENABLE_BUZZING_INIT|ENABLE_OTA_UPDATE|USE_ESP_IDF_LOG),
   *     "esp32_ver": "v4.4.1-1-gb8050b365e",
   *     "arduino_ver": "2.0.4",
   *     "ota_parts_used": "x2, R0, B0, U1",
   *     "partitions": [
   *       {
   *         "part": "app0:0x10000:160000",
   *         "ota_state": "0xffffffff",
   *         "app_desc": "jrcmatic(user@host):jrc-v0.0.3-83-gae610e9-dirty",
   *         "build_date": "06 Jul 2022, 15:22:57+0300",
   *         "part_sha256": "278d86c2"
   *       },
   *       {
   *         "part": "app1:0x170000:160000",
   *         "ota_state": "0xffffffff",
   *         "app_desc": "",
   *         "build_date": "",
   *         "part_sha256": ""
   *       }
   *     ]
   *   },
   *   "node_state": {
   *     "last_boot_reason": 1,
   *     "part_size": 1441792,
   *     "sketch_size": 883760,
   *     "part_use": 61.29594282670455,
   *     "heap_size": 120652,
   *     "heap_max_used": 4294899556,
   *     "heap_max_use": 3559741.7000961443,
   *     "heap_free_min": 188392,
   *     "esp_get_free_heap_size": 192816,
   *     "esp_get_free_internal_heap_size": 192668,
   *     "esp_get_minimum_free_heap_size": 187328,
   *     "ESP_getHeapSize": 339596,
   *     "ESP_getFreeHeap": 254600,
   *     "ESP_getMinFreeHeap": 249224,
   *     "ESP_getMaxAllocHeap": 110580,
   *     "def_total_allocated_bytes": 84012,
   *     "def_total_free_bytes": 193752,
   *     "def_minimum_free_bytes": 188392,
   *     "def_largest_free_block": 110580
   *   },
   *   "config": {
   *     "serial_autoconf_timeout": 0,
   *     "log_level_run": 3,
   *     "log_level_build": 3,
   *     "log_sink": 1,
   *     "log_sink_fpath": "/logs.txt",
   *     "log_sink_disk_usage_purge_prcnt": 0.8999999761581421,
   *     "log_sink_sync_interval_ms": 3141,
   *     "nslots": 256,
   *     "slot_len": 180,
   *     "serialize_len": 1024,
   *     "storage": 0,
   *     "gnss": 1,
   *     "ota_url": "https://some.host.com/and/path",
   *     "ota_update_cert_pem_len": 1234,
   *     "net_dev": 4,
   *     "wifi_ssd": "***",
   *     "wifi_pwd": "***",
   *     "cell_apn": "***",
   *     "sim_card_pin": "",
   *     "srv_proto": 1,
   *     "srv_host": "some.host.com",
   *     "srv_path": "/hub/api",
   *     "srv_port": 8081,
   *     "net_recv_timeout": 5000,
   *     "srv_sync_timeout": 120,
   *     "net_retries": 5,
   *     "net_udp_reconnect_delay": 3000,
   *     "stationary_timeout_vals": [
   *       20,
   *       40,
   *       60
   *     ],
   *     "data_interval_vals": [
   *       1000,
   *       2000,
   *       5000
   *     ],
   *     "obfcm_interval": 120000,
   *     "obd_max_errors": 3,
   *     "ping_back_interval": 900,
   *     "wakeup_reset": 0,
   *     "wakeup_motion_thr": 0.4000000059604645,
   *     "wakeup_jumpstart_thr": 13.600000381469727,
   *     "cool_temp": 80.0,
   *     "cool_delay": 5,
   *     "pin_sensor1": 34,
   *     "pin_sensor2": 26
   *   }
   * }
   * ```
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
   * NOTE: teach platformIO's monitor-filter about new macroflags.
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
  /** NOTE: remember to update also `macroflags.py` monitor-filter. */
  int serial_autoconf_timeout{CONFIG_MODE_TIMEOUT};
  uint8_t log_level_build{CORE_DEBUG_LEVEL};
  LogLevels log_levels{RUNTIME_LOG_LEVELS};
  uint8_t log_sink{LOG_SINK};
  const char *log_sink_fpath{LOG_SINK_FPATH};
  float log_sink_disk_usage_purge_prcnt{LOG_SINK_DISK_USAGE_PURGE_RATIO};
  int32_t log_sink_sync_interval_ms{LOG_SINK_SYNC_INTERVAL_MS};
  int nslots{BUFFER_SLOTS};
  int slot_len{BUFFER_LENGTH};
  int serialize_len{SERIALIZE_BUFFER_SIZE};
  /** Bit for enable/disable decided GLOBALon compile-time.  */
  uint8_t storage{STORAGE};
  uint8_t gnss{GNSS};
  const char *ota_url{OTA_UPDATE_URL};
  const char *ota_update_cert_pem{OTA_UPDATE_CERT_PEM};
  uint8_t net_dev{NET_DEVICE};
  const char *wifi_ssd{WIFI_SSID};
  const char *wifi_pwd{WIFI_PASSWORD};
  const char *cell_apn{CELL_APN};
  const char *sim_card_pin{SIM_CARD_PIN};
  uint8_t srv_proto{SERVER_PROTOCOL};
  const char *srv_host{SERVER_HOST};
  const char *srv_path{SERVER_PATH};
  uint16_t srv_port{SERVER_PORT};
  uint32_t net_recv_timeout{DATA_RECEIVING_TIMEOUT};
  uint32_t srv_sync_timeout{SERVER_SYNC_INTERVAL};
  uint8_t net_retries{NET_CONNECT_RETRIES};
  uint16_t net_udp_reconnect_delay{UDP_CONNECT_RETRY_DELAY_MS};
  uint16_t stationary_timeout_vals[3]{STATIONARY_TIME_TABLE};
  uint16_t data_interval_vals[3]{DATA_INTERVAL_TABLE};
  uint32_t obfcm_interval{OBFCM_INTERVAL_MS};
  uint8_t obd_max_errors{MAX_OBD_ERRORS};
  uint16_t ping_back_interval{PING_BACK_INTERVAL};
  uint8_t wakeup_reset{RESET_AFTER_WAKEUP};
  float wakeup_motion_thr{MOTION_THRESHOLD};
  float wakeup_jumpstart_thr{THR_VOLTAGE};
  float cool_temp{COOLING_DOWN_TEMP};
  uint16_t cool_delay{COOLING_DOWN_SLEEP_SEC};
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
