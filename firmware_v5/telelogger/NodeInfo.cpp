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


void to_json(Json& j, const stationary_interval_t& t) {
  j = Json{ t.stationary_duration_sec, t.transmission_interval_ms };
}

void from_json(const Json& j, stationary_interval_t& t) {
  j[0].get_to(t.stationary_duration_sec);
  j[1].get_to(t.transmission_interval_ms);
}


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
        precs.push_back(PartRec{part, desc, state});
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
      if (part->address == prec.part->address) return i;
      i++;
    }
  }
  return -1;
}


Json _partition_record_to_json(const PartRec &prec) {
    const esp_partition_t *part = prec.part;
    const esp_app_desc_t desc = prec.desc;
    const esp_ota_img_states_t state = prec.state;

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
      // (see https://json.Json.me/api/basic_json/error_handler_t/)
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
Json node_info_t::hw_info_to_json() const {
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

Json node_info_t::fw_info_to_json(const PartInfos precs) const {
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

  auto partitions = Json::array();
  for (const auto &prec : precs){
    partitions.push_back(_partition_record_to_json(prec));
  }

  return {
      {"macroflags", ss_macroflags.str()},
      {"arduino_ver", ss_arduino_ver.str()},
      {"esp32_ver", IDF_VER},
      {"ota_parts_used", ss_parts_used.str()},
      {"partitions", partitions},
      {"partition_size", partition_size},
      {"sketch_size", sketch_size},
      {"partition_use", partition_use},
      {"log_level_build", log_level_build},
      {"nslots", nslots},
      {"slot_len", slot_len},
      {"serialize_len", serialize_len},
      {"storage", storage},
      {"gnss", gnss},
      {"srv_proto", srv_proto},
  };
}  // fw_info_to_json()

Json node_info_t::node_state_to_json() const {
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

  unsigned long wake_sec_now = (millis() - wakeup_tstamp) / 1000;

  return {
  {"last_boot_reason", esp_reset_reason()},
  {"reboots", boot_ark.reboots},
  {"naps", boot_ark.naps},
  {"nap_sec", boot_ark.nap_sec},
  {"wake_sec", boot_ark.wake_sec + wake_sec_now},
  {"nap_ratio", (float) boot_ark.nap_sec / (boot_ark.nap_sec +
      boot_ark.wake_sec + wake_sec_now)},

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


#if HIDE_SECRETS_IN_LOGS
static void _hide_sensitive_configs(Json &cfg) {
  constexpr const char *mask = "***";
  cfg.update(
      {
          {"wifi_pwd", mask},
          {"cell_apn", (const char *)apn2log},
          {"sim_card_pin", mask},
          {"srv_host", (const char *)host2log},
          {"srv_path", mask},
          {"srv_port", 0},
          {"ota_url", (const char *)ota_url2log},
      },
      /* (recursively) merge_objects? */ false);
}  // _hide_sensitive_configs()
#endif  // HIDE_SECRETS_IN_LOGS


Json node_info_t::config_to_json() const {
  Json cfg{
      {"obd_pipe_sec", obd_pipe_sec},
      {"log_levels", log_levels},
      {"log_sink", log_sink},
      {"log_sink_fpath", log_sink_fpath},
      {"log_sink_disk_usage_purge_prcnt", log_sink_disk_usage_purge_prcnt},
      {"log_sink_sync_interval_ms", log_sink_sync_interval_ms},
      {"ota_url", ota_url},
      {"ota_update_cert_pem_len", std::strlen(ota_update_cert_pem)},
      {"net_dev", net_dev},
      {"wifi_ssd", wifi_ssd},
      {"wifi_pwd", wifi_pwd},
      {"cell_apn", cell_apn},
      {"sim_card_pin", sim_card_pin},
      {"srv_host", srv_host},
      {"srv_path", srv_path},
      {"srv_port", srv_port},
      {"net_recv_timeout_ms", net_recv_timeout_ms},
      {"srv_sync_timeout_ms", srv_sync_timeout_ms},
      {"net_retries", net_retries},
      {"net_udp_reconnect_delay_ms", net_udp_reconnect_delay_ms},
      {"transmission_intervals", transmission_intervals},
      {"obfcm_interval", obfcm_interval},
      {"obd_max_errors", obd_max_errors},
      {"ping_back_interval_sec", ping_back_interval_sec},
      {"reboot_on_wakeup", reboot_on_wakeup},
      {"wakeup_motion_thr", wakeup_motion_thr},
      {"wakeup_jumpstart_thr", wakeup_jumpstart_thr},
      {"cool_temp", cool_temp},
      {"cool_delay_sec", cool_delay_sec},
      {"pin_sensor1", pin_sensor1},
      {"pin_sensor2", pin_sensor2},
  };

#if HIDE_SECRETS_IN_LOGS
  _hide_sensitive_configs(cfg);
#endif // HIDE_SECRETS_IN_LOGS

  return cfg;
}  // config_to_json()


Json node_info_t::to_json() const {
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
        {"config", config_to_json()},
    };
}


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
