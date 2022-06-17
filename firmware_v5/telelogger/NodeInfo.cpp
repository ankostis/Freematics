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

#include <Esp.h>
#include <esp_app_format.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <soc/rtc.h>

#include <string>
#include <cstring>
#if BOARD_HAS_PSRAM && BOARD_HAS_PSRAM_HIGH
#include "esp32/himem.h"
#endif
#include "NodeInfo.h"
#include "json.hpp"


void mac_to_device_id(const uint64_t mac, char *device_id) {
  uint64_t seed = mac >> 8;
  for (int i = 0; i < 8; i++, seed >>= 5) {
    byte x = (byte)seed & 0x1f;
    if (x >= 10) {
      x = x - 10 + 'A';
      switch (x) {
        case 'B':
          x = 'W';
          break;
        case 'D':
          x = 'X';
          break;
        case 'I':
          x = 'Y';
          break;
        case 'O':
          x = 'Z';
          break;
      }
    } else {
      x += '0';
    }
    device_id[i] = x;
  }
  device_id[8] = 0;
}

std::string generate_node_infos() {
  char devid[12];

  uint64_t mac = ESP.getEfuseMac();
  mac_to_device_id(mac, devid);
  uint32_t PartitionSize = esp_ota_get_running_partition()->size; // +395 from build report
  uint32_t SketchSize = ESP.getSketchSize();

  uint32_t HeapSize = ESP.getHeapSize();
  uint32_t HeapFree = ESP.getFreeHeap();
  uint32_t HeapUsed = HeapSize - HeapFree;

#if BOARD_HAS_PSRAM
  uint32_t PsramSize = ESP.getPsramSize();
  uint32_t PsramFree = ESP.getFreePsram();
  uint32_t PsramUsed =  PsramSize - PsramFree ;
#if BOARD_HAS_PSRAM_HIGH
  size_t PsramHighSize = esp_himem_get_phys_size();
  size_t PsramHighFree = esp_himem_get_free_size();
  size_t PsramHighUsed = PsramHighSize - PsramHighFree;
#endif
#endif
  char buf[1024];

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
  sprintf(buf,
      "{"
      "\n   \"device_id\": \"%s\""
      ",\n      \"board\": \"%s-v%i@%ix%i\""
      ",\n        \"mac\": \"0x%llX\""
      ",\n      \"flash\": \"%iMiB@%iMHz\""
      ",\n   \"slow_rtc\": \"%i@%iKHz\""
      ",\n  \"esp32_sdk\": \"%s\""
      ",\n    \"arduino\": \"%i.%i.%i\""
      ",\n \"fw_version\": \"%s\""
      ",\n \"sketch_elf\": \"%.7s-%lx\""  // sha256 is 32-bytes (not just 4)
      ",\n \"build_date\": \"%s\""
      ",\n   \"build_by\": \"%s@%s\""
      ",\n  \"last_boot\": %i"
      ",\n    \"partition_size\": %u"
      ",\n       \"sketch_size\": %u"
      ",\n     \"partition_use\": %.2f"
      ",\n\"ota_partition_size\": %u"
      ",\n       \"heap\": %u"
      ",\n  \"heap_used\": %u"
      ",\n   \"heap_use\": %.2f"
#if BOARD_HAS_PSRAM
      ",\n      \"psram\": %u"
      ",\n \"psram_used\": %u"
      ",\n  \"psram_use\": %.2f"
#   if BOARD_HAS_PSRAM_HIGH
      ",\n     \"psramh\": %u"
      ",\n\"psramh_used\": %u"
      ",\n \"psramh_use\": %.2f"
#   endif
#endif
      "\n}"
      , devid
      , ESP.getChipModel()
      , ESP.getChipRevision()
      , ESP.getCpuFreqMHz()
      , ESP.getChipCores()
      , ESP.getEfuseMac()
      , ESP.getFlashChipSize() >> 20
      , ESP.getFlashChipSpeed() / 1000000
      , rtc_clk_slow_freq_get()
      , rtc_clk_slow_freq_get_hz() / 1000
      , IDF_VER
      , ESP_ARDUINO_VERSION_MAJOR
      , ESP_ARDUINO_VERSION_MINOR
      , ESP_ARDUINO_VERSION_PATCH
      , GIT_DESCRIBE
      , ESP.getSketchMD5().c_str()
      , esp_ota_get_app_description()->app_elf_sha256
      , __DATE__ " " __TIME__
      , BUILD_USERNAME
      , BUILD_HOSTNAME
      , esp_reset_reason()
      , PartitionSize
      , SketchSize
      , (100.0 * SketchSize / PartitionSize)
      // NOTE: `ESP.getFreeSketchSpace()` brings next (OTA) partition's size
      // (see espressif/arduino-esp32#3501
      , ESP.getFreeSketchSpace()
      , HeapSize
      , HeapUsed
      , (100.0 * HeapUsed / HeapSize)
#if BOARD_HAS_PSRAM
      , PsramSize  // not exactly 4MiB reported
      , PsramUsed
      , (100.0 * PsramUsed / PsramSize)
#   if BOARD_HAS_PSRAM_HIGH
      , PsramHighSize
      , PsramHighUsed
      , (100.0 * PsramHighUsed / PsramHighSize)
#   endif  // BOARD_HAS_PSRAM_HIGH
#endif  // BOARD_HAS_PSRAM
  );
#pragma GCC diagnostic pop

#if BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
  ESP_LOGI(TAG, "Writing PSRAM to validate it...");
  uint32_t *ptr = (uint32_t *)ps_malloc(PsramSize);
  if (!ptr) {
    ESP_LOGE(TAG, "unable to allocate %i bytes", PsramSize);
  } else {
    uint32_t t = millis();
    for (int i = 0; i < PsramSize / 4; i++) {
      ptr[i] = 0xa5a5a5a5;
    }
    ESP_LOGI(TAG, "OK @%iKB/s", PsramSize / (millis() - t));
  }
  ESP_LOGI(TAG, "Verifying PSRAM...");
  int errors = 0;
  uint32_t t = millis();
  for (int i = 0; i < PsramSize / 4; i++) {
    if (ptr[i] != 0xa5a5a5a5) {
      ESP_LOGE(TAG, "mismatch @ 0x%i", i * 4, 16);
      errors++;
    }
  }
  if (errors == 0) {
    ESP_LOGI(TAG, "OK @%iKB/s", PsramSize / (millis() - t));
  }
  free(ptr);
#endif  // BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE

  return std::string{buf};
}

nlohmann::json node_info_to_json(const node_info_t &infos) {
  nlohmann::json root;

  root["device_id"] = infos.device_id;
  root["vin"] = infos.vin;
  root["enable_flags"] = infos.enable_flags;
  root["serial_autoconf_timeout"] = infos.serial_autoconf_timeout;
  root["log_level_run"] = infos.log_level_run;
  root["log_level_build"] = infos.log_level_build;
  root["log_sink"] = infos.log_sink;
  root["log_sink_fpath"] = infos.log_sink_fpath;
  root["log_sink_disk_usage_purge_prcnt"] = infos.log_sink_disk_usage_purge_prcnt;
  root["log_sink_sync_interval_ms"] = infos.log_sink_sync_interval_ms;
  root["nslots"] = infos.nslots;
  root["slot_len"] = infos.slot_len;
  root["serialize_len"] = infos.serialize_len;
  root["storage"] = infos.storage;
  root["gnss"] = infos.gnss;
  root["ota_url"] = infos.ota_url;
  root["ota_update_cert_pem_len"] = std::strlen(infos.ota_update_cert_pem);
  root["net_dev"] = infos.net_dev;
  root["wifi_ssd"] = infos.wifi_ssd;
  root["wifi_pwd"] = infos.wifi_pwd;
  root["cell_apn"] = infos.cell_apn;
  root["sim_card_pin"] = infos.sim_card_pin;
  root["srv_proto"] = infos.srv_proto;
  root["srv_host"] = infos.srv_host;
  root["srv_path"] = infos.srv_path;
  root["srv_port"] = infos.srv_port;
  root["net_recv_timeout"] = infos.net_recv_timeout;
  root["srv_sync_timeout"] = infos.srv_sync_timeout;
  root["net_retries"] = infos.net_retries;
  root["net_udp_reconnect_delay"] = infos.net_udp_reconnect_delay;
  // root["stationary_timeout_vals"] = infos.stationary_timeout_vals;
  root["obfcm_interval"] = infos.obfcm_interval;
  root["obd_max_errors"] = infos.obd_max_errors;
  root["ping_back_interval"] = infos.ping_back_interval;
  root["wakeup_reset"] = infos.wakeup_reset;
  root["wakeup_motion_thr"] = infos.wakeup_motion_thr;
  root["wakeup_jumpstart_thr"] = infos.wakeup_jumpstart_thr;
  root["cool_temp"] = infos.cool_temp;
  root["cool_delay"] = infos.cool_delay;
  root["pin_sensor1"] = infos.pin_sensor1;
  root["pin_sensor2"] = infos.pin_sensor2;

  return root;
}