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

#include "config.h"
#include "NodeInfo.h"
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
#include <sstream>
#include <tuple>
#include <vector>

#if BOARD_HAS_PSRAM && BOARD_HAS_PSRAM_HIGH
#include "esp32/himem.h"
#endif


std::string  mac_to_device_id(const uint64_t mac) {
  std::stringstream ss{};
  uint64_t seed = mac >> 8;
  for (int i = 0; i < 8; i++, seed >>= 5) {
    uint8_t x = (uint8_t)seed & 0x1f;
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
    ss << x;
  }

  return ss.str();
}


node_info_t::node_info_t():
  mac(ESP.getEfuseMac()),
  device_id(mac_to_device_id(mac)),

  /** ATTENTION: increase `macroflags` size if more macroflags added. */
  /** NOTE: teach platformIO's monitor-filter about new macroflags. */
  macroflags(
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
  ),
  partition_size(esp_ota_get_running_partition()->size),
  sketch_size(ESP.getSketchSize()),
  partition_use(100.0 * sketch_size / partition_size),
  boot_heap_size(ESP.getHeapSize()),

  vin{},

  serial_autoconf_timeout(CONFIG_MODE_TIMEOUT),
  log_level_run(RUNTIME_ALL_TAGS_LOG_LEVEL),
  log_level_build(CORE_DEBUG_LEVEL),
  log_sink(LOG_SINK),
  log_sink_fpath(LOG_SINK_FPATH),
  log_sink_disk_usage_purge_prcnt(LOG_SINK_DISK_USAGE_PURGE_RATIO),
  log_sink_sync_interval_ms(LOG_SINK_SYNC_INTERVAL_MS),
  nslots(BUFFER_SLOTS),
  slot_len(BUFFER_LENGTH),
  serialize_len(SERIALIZE_BUFFER_SIZE),
  storage(STORAGE),
  gnss(GNSS),
  ota_url(OTA_UPDATE_URL),
  ota_update_cert_pem(OTA_UPDATE_CERT_PEM),
  net_dev(NET_DEVICE),
  wifi_ssd{WIFI_SSID},
  wifi_pwd{WIFI_PASSWORD},
  cell_apn{CELL_APN},
  sim_card_pin{SIM_CARD_PIN},
  srv_proto(SERVER_PROTOCOL),
  srv_host{SERVER_HOST},
  srv_path{SERVER_PATH},
  srv_port(SERVER_PORT),
  net_recv_timeout(DATA_RECEIVING_TIMEOUT),
  srv_sync_timeout(SERVER_SYNC_INTERVAL),
  net_retries(NET_CONNECT_RETRIES),
  net_udp_reconnect_delay(UDP_CONNECT_RETRY_DELAY_MS),
  stationary_timeout_vals STATIONARY_TIME_TABLE,
  data_interval_vals DATA_INTERVAL_TABLE,
  obfcm_interval(OBFCM_INTERVAL_MS),
  obd_max_errors(MAX_OBD_ERRORS),
  ping_back_interval(PING_BACK_INTERVAL),
  wakeup_reset(RESET_AFTER_WAKEUP),
  wakeup_motion_thr(MOTION_THRESHOLD),
  wakeup_jumpstart_thr(THR_VOLTAGE),
  cool_temp(COOLING_DOWN_TEMP),
  cool_delay(COOLING_DOWN_SLEEP_SEC),
  pin_sensor1(PIN_SENSOR1),
  pin_sensor2(PIN_SENSOR2)
 {};


const PartInfos collect_ota_partition_records() {
  esp_partition_iterator_t it = esp_partition_find(
      ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  PartInfos precs;
  while (it) {
    const esp_partition_t *part{};
    if ((part = esp_partition_get(it))) {
      esp_ota_img_states_t state{};
      if (esp_ota_get_state_partition(part, &state) == ESP_OK) {
        esp_app_desc_t desc{};
        // Err ignored, check magic instead.
        esp_ota_get_partition_description(part, &desc);
        precs.push_back(PartRec(part, desc, state));
      }
    }
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  return precs;
}  // collect_ota_partition_records()


static int ota_partition_index(const esp_partition_t *part, const PartInfos &precs) {
  if (part) {
    int i = 0;
    for (const auto &prec : precs) {
      if (part->address == std::get<0>(prec)->address) return i;
      i++;
    }
  }
  return -1;
}


nlohmann::ordered_json _partition_record_to_json(const PartRec &prec) {
    const esp_partition_t *part = std::get<0>(prec);
    const esp_app_desc_t desc = std::get<1>(prec);
    const esp_ota_img_states_t state = std::get<2>(prec);

    std::stringstream ss_ota_addr;
    ss_ota_addr << part->label << ':' << "0x" << std::hex << part->address;
    ss_ota_addr << ':' << part->size;

    std::stringstream ss_ota_state;
    ss_ota_state << "0x" << std::hex << state;

    std::stringstream ss_app{};
    std::stringstream ss_date{};
    std::stringstream ss_sha256{};
    if (desc.magic_word == ESP_APP_DESC_MAGIC_WORD) {

      // Possibly non-UTF8 app-name/ver from bad partitions,
      // json will crash on `dump()` unless `dump(error_handler=ignore/replace)`
      // (see https://json.nlohmann.me/api/basic_json/error_handler_t/)
      //
      ss_app << desc.project_name << ':' << desc.version;
      ss_date << desc.date << ", " << desc.time;

      if (part->type == PART_TYPE_APP) {
        // Adapted from `esp_ota_get_app_elf_sha256()`, to avoid sha256-recalc
        // and read it from the (already validated) end of the partition image.
        //
        esp_image_metadata_t part_meta;
        const esp_partition_pos_t part_pos{part->address, part->size};
        if (
          esp_image_get_metadata(&part_pos, &part_meta) == ESP_OK &&
          part_meta.image.hash_appended
        )
          ncat_hex(ss_sha256, part_meta.image_digest, 4);
      }  // endif invalid `esp_app_desc_t` struct
    }


    return {
      {"part", ss_ota_addr.str()},
      {"ota_state", ss_ota_state.str()},
      {"app_desc", ss_app.str()},
      {"build_date", ss_date.str()},
      {"part_sha256", ss_sha256.str()},
    };

}  // _partition_record_to_json()


/**
 * - Private but non-static, to be testable.
 *
 * ATTENTION: rev1 devices (like the sample above), need the PSRAM workaround
 * (`-mfix-esp32-psram-cache-issue`) if `-DBOARD_HAS_PSRAM` build-flag enabled.
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html#esp32-rev-v1
 */
nlohmann::ordered_json node_info_t::hw_info_to_json() const {
  std::stringstream ss_mac;
  ncat_hex(ss_mac, (const uint8_t *)(&mac), 6);

  return {
      {"board", std::string{ESP.getChipModel()} + "-v" +
                    std::to_string(ESP.getChipRevision())},
      {"cpu", std::to_string(ESP.getChipCores()) + "x" +
                  std::to_string(ESP.getCpuFreqMHz()) + "MHz"},
      {"mac", ss_mac.str()},
      {"flash", std::to_string(ESP.getFlashChipSize() >> 20) + "MiB@" +
                    std::to_string(ESP.getFlashChipSpeed() / 1000000) + "MHz"},
      {"slow_rtc", std::to_string(rtc_clk_slow_freq_get()) + "@" +
                       std::to_string(rtc_clk_slow_freq_get_hz() / 1000) +
                       "KHz"},
  };
}  // hw_info_to_json()

nlohmann::ordered_json node_info_t::fw_info_to_json(const PartInfos precs) const {
  const esp_partition_t *run_part = esp_ota_get_running_partition();
  const esp_partition_t *boot_part = esp_ota_get_boot_partition();
  const esp_partition_t *upd_part = esp_ota_get_next_update_partition(run_part);

  std::stringstream ss_parts_used;
  // "ota_parts": "x%i, R%i, B%i, U%i"
  ss_parts_used << "x" << (int)esp_ota_get_app_partition_count()
                << ", R" << ota_partition_index(run_part, precs)
                << ", B" << ota_partition_index(boot_part, precs)
                << ", U" << ota_partition_index(upd_part, precs);

  std::stringstream ss_arduino_ver;
  ss_arduino_ver << ESP_ARDUINO_VERSION_MAJOR << "."
                 << ESP_ARDUINO_VERSION_MINOR << "."
                 << ESP_ARDUINO_VERSION_PATCH;

  std::stringstream ss_macroflags;
  ss_macroflags << "0x" << std::hex << macroflags;

  auto partitions = nlohmann::ordered_json::array();
  for (const auto &prec : precs){
    partitions.push_back(_partition_record_to_json(prec));
  }

  return {
      {"macroflags", ss_macroflags.str()},
      {"esp32_ver", IDF_VER},
      {"arduino_ver", ss_arduino_ver.str()},
      {"ota_parts_used", ss_parts_used.str()},
      {"partitions", partitions},
  };
}  // fw_info_to_json()

nlohmann::ordered_json node_info_t::node_state_to_json() const {
  // TODO: root["boot_count"] = boot_count
  const uint32_t heap_size = ESP.getHeapSize();
  const uint32_t MinHeapFree = esp_get_minimum_free_heap_size();
  const int32_t MaxHeapUsed = heap_size - MinHeapFree;
  multi_heap_info_t heap_info;
  heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

#if BOARD_HAS_PSRAM
  // const uint32_t psram_size = esp_spiram_get_size();  // FIXME: CAPs for RTC & SPIRAM
  const uint32_t psram_size = ESP.getPsramSize();
  const uint32_t psram_free = ESP.getFreePsram();
  const int32_t psram_used = psram_size - psram_free;
#if BOARD_HAS_PSRAM_HIGH
  const uint32_t psramh_size = esp_himem_get_phys_size();
  const size_t psramh_free = esp_himem_get_free_size();
  const int32_t psramh_used = psramh_size - psramh_free;
#endif
#endif

  return {
  {"last_boot_reason", esp_reset_reason()},

  {"partition_size", partition_size},
  {"sketch_size", sketch_size},
  {"partition_max_use", partition_use},

  {"boot_heap_size", boot_heap_size},
  {"heap_size", heap_size},
  {"heap_max_used", MaxHeapUsed},
  {"heap_max_use", (100.0 * MaxHeapUsed / heap_size)},
  {"heap_free_min", MinHeapFree},
  {"esp_get_free_heap_size", esp_get_free_heap_size()},
  {"esp_get_free_internal_heap_size", esp_get_free_internal_heap_size()},
  {"ESP_getFreeHeap", ESP.getFreeHeap()},
  {"ESP_getMinFreeHeap", ESP.getMinFreeHeap()},
  {"ESP_getMaxAllocHeap", ESP.getMaxAllocHeap()},

  {"def_total_allocated", heap_info.total_allocated_bytes},
  {"def_total_free", heap_info.total_free_bytes},
  {"def_minimum_free", heap_info.minimum_free_bytes},
  {"def_largest_free_block", heap_info.largest_free_block},

#if BOARD_HAS_PSRAM
    {"psram", psram_size},
    {"psram_used", psram_used},
    {"psram_use", (100.0 * psram_used / psram_size)},

#if BOARD_HAS_PSRAM_HIGH
  {"psramh", psramh_size},
  {"psramh_used", psramh_used},
  {"psramh_use", (100.0 * psramh_used / psramh_size)},
#endif
#endif
  };
}  // node_state_to_json()


nlohmann::ordered_json node_info_t::to_json() const {
  nlohmann::ordered_json cfg{
      {"serial_autoconf_timeout", serial_autoconf_timeout},
      {"log_level_run", log_level_run},
      {"log_level_build", log_level_build},
      {"log_sink", log_sink},
      {"log_sink_fpath", log_sink_fpath},
      {"log_sink_disk_usage_purge_prcnt", log_sink_disk_usage_purge_prcnt},
      {"log_sink_sync_interval_ms", log_sink_sync_interval_ms},
      {"nslots", nslots},
      {"slot_len", slot_len},
      {"serialize_len", serialize_len},
      {"storage", storage},
      {"gnss", gnss},
      {"ota_url", ota_url},
      {"ota_update_cert_pem_len", std::strlen(ota_update_cert_pem)},
      {"net_dev", net_dev},
      {"wifi_ssd", wifi_ssd},
      {"wifi_pwd", wifi_pwd},
      {"cell_apn", cell_apn},
      {"sim_card_pin", sim_card_pin},
      {"srv_proto", srv_proto},
      {"srv_host", srv_host},
      {"srv_path", srv_path},
      {"srv_port", srv_port},
      {"net_recv_timeout", net_recv_timeout},
      {"srv_sync_timeout", srv_sync_timeout},
      {"net_retries", net_retries},
      {"net_udp_reconnect_delay", net_udp_reconnect_delay},
      {"stationary_timeout_vals", stationary_timeout_vals},
      {"data_interval_vals", data_interval_vals},
      {"obfcm_interval", obfcm_interval},
      {"obd_max_errors", obd_max_errors},
      {"ping_back_interval", ping_back_interval},
      {"wakeup_reset", wakeup_reset},
      {"wakeup_motion_thr", wakeup_motion_thr},
      {"wakeup_jumpstart_thr", wakeup_jumpstart_thr},
      {"cool_temp", cool_temp},
      {"cool_delay", cool_delay},
      {"pin_sensor1", pin_sensor1},
      {"pin_sensor2", pin_sensor2},
  };

  const PartInfos precs = collect_ota_partition_records();
  const auto fw_j = fw_info_to_json(precs);

  //// Place firmware name/version & build-date
  //   at the top of the JSON.
  //
  const auto my_part = esp_ota_get_running_partition();
  const int pi = ota_partition_index(my_part, precs);
  std::string app_desc;
  std::string build_date;
  if (pi >= 0) {
    const auto part_j = fw_j["partitions"].at(pi);
    app_desc = part_j.value("app_desc", "");
    build_date = part_j.value("build_date", "");
  }

  return {
        {"device_id", device_id},
        {"vin", vin},
        {"app_desc", app_desc},
        {"build_date", build_date},
        {"node_hw", hw_info_to_json()},
        {"node_fw", fw_j},
        {"node_state", node_state_to_json()},
        {"config", cfg},
    };
}

#if HIDE_SECRETS_IN_LOGS
void hide_sensitive_node_infos(nlohmann::ordered_json &infos) {
  constexpr const char *mask = "***";
  infos.update(
      {
          {"config",
           {
               {"wifi_pwd", mask},
               {"cell_apn", apn2log},
               {"sim_card_pin", mask},
               {"srv_host", host2log},
               {"srv_path", mask},
               {"srv_port", 0},
               {"ota_url", ota_url2log},
           }},
      },
      /* (recursively) merge_objects */ true);
}  // hide_sensitive_node_infos()
#endif // HIDE_SECRETS_IN_LOGS


#if BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
void validate_psram_can_indeed_write() {
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
}  // validate_psram_can_indeed_write()
#endif  // BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
