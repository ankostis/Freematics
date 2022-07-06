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
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <bootloader_common.h>
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
nlohmann::ordered_json _hw_info_as_json() {
  std::stringstream ss_mac;
  uint8_t mac_buf[8]{};
  esp_efuse_mac_get_default(mac_buf);
  cat_hex(ss_mac, mac_buf);

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
}  // _hw_info_as_json()

nlohmann::ordered_json _fw_info_as_json(const macroflags_t macroflags,
                                        const PartInfos precs) {
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
}  // _fw_info_as_json()


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


nlohmann::ordered_json node_status_to_json(const node_info_t &infos) {
  // TODO: root["boot_count"] = boot_count
  const uint32_t SketchSize = ESP.getSketchSize();
  const uint32_t HeapFree = esp_get_minimum_free_heap_size();
  const uint32_t HeapUsed = infos.heap_size - HeapFree;
  multi_heap_info_t heap_info;
  heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

#if BOARD_HAS_PSRAM
    const uint32_t psram_free = ESP.getFreePsram();
    const uint32_t psram_used = infos.psram_size - psram_free;
#if BOARD_HAS_PSRAM_HIGH
  const size_t psramh_free = esp_himem_get_free_size();
  const size_t psramh_used = infos.psramh_size - psramh_free;
#endif
#endif

  return {
  {"last_boot_reason", esp_reset_reason()},

  {"part_size", infos.partition_size},
  {"sketch_size", SketchSize},
  {"part_use", (100.0 * SketchSize / infos.partition_size)},

  {"heap_size", infos.heap_size},
  {"heap_max_used", HeapUsed},
  {"heap_max_use", (100.0 * HeapUsed / infos.heap_size)},
  {"heap_free_min", HeapFree},
  {"esp_get_free_heap_size", esp_get_free_heap_size()},
  {"esp_get_free_internal_heap_size", esp_get_free_internal_heap_size()},
  {"esp_get_minimum_free_heap_size", esp_get_minimum_free_heap_size()},
  {"ESP_getHeapSize", ESP.getHeapSize()},
  {"ESP_getFreeHeap", ESP.getFreeHeap()},
  {"ESP_getMinFreeHeap", ESP.getMinFreeHeap()},
  {"ESP_getMaxAllocHeap", ESP.getMaxAllocHeap()},

  {"def_total_allocated_bytes", heap_info.total_allocated_bytes},
  {"def_total_free_bytes", heap_info.total_free_bytes},
  {"def_minimum_free_bytes", heap_info.minimum_free_bytes},
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
}  // node_status_to_json()


nlohmann::ordered_json node_info_to_json(const node_info_t &info) {
  nlohmann::ordered_json cfg{
      {"serial_autoconf_timeout", info.serial_autoconf_timeout},
      {"log_level_run", info.log_level_run},
      {"log_level_build", info.log_level_build},
      {"log_sink", info.log_sink},
      {"log_sink_fpath", info.log_sink_fpath},
      {"log_sink_disk_usage_purge_prcnt", info.log_sink_disk_usage_purge_prcnt},
      {"log_sink_sync_interval_ms", info.log_sink_sync_interval_ms},
      {"nslots", info.nslots},
      {"slot_len", info.slot_len},
      {"serialize_len", info.serialize_len},
      {"storage", info.storage},
      {"gnss", info.gnss},
      {"ota_url", info.ota_url},
      {"ota_update_cert_pem_len", std::strlen(info.ota_update_cert_pem)},
      {"net_dev", info.net_dev},
      {"wifi_ssd", info.wifi_ssd},
      {"wifi_pwd", info.wifi_pwd},
      {"cell_apn", info.cell_apn},
      {"sim_card_pin", info.sim_card_pin},
      {"srv_proto", info.srv_proto},
      {"srv_host", info.srv_host},
      {"srv_path", info.srv_path},
      {"srv_port", info.srv_port},
      {"net_recv_timeout", info.net_recv_timeout},
      {"srv_sync_timeout", info.srv_sync_timeout},
      {"net_retries", info.net_retries},
      {"net_udp_reconnect_delay", info.net_udp_reconnect_delay},
      {"stationary_timeout_vals", info.stationary_timeout_vals},
      {"data_interval_vals", info.data_interval_vals},
      {"obfcm_interval", info.obfcm_interval},
      {"obd_max_errors", info.obd_max_errors},
      {"ping_back_interval", info.ping_back_interval},
      {"wakeup_reset", info.wakeup_reset},
      {"wakeup_motion_thr", info.wakeup_motion_thr},
      {"wakeup_jumpstart_thr", info.wakeup_jumpstart_thr},
      {"cool_temp", info.cool_temp},
      {"cool_delay", info.cool_delay},
      {"pin_sensor1", info.pin_sensor1},
      {"pin_sensor2", info.pin_sensor2},
  };

  const PartInfos precs = collect_ota_partition_records();
  const auto fw_j = _fw_info_as_json(info.macroflags, precs);

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
        {"device_id", info.device_id},
        {"vin", info.vin},
        {"app_desc", app_desc},
        {"build_date", build_date},
        {"vin", info.vin},
        {"node_hw", _hw_info_as_json()},
        {"node_fw", fw_j},
        {"node_state", node_status_to_json(info)},
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
