
/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/

#include "config.h"
#include <unity.h>

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

#include <esp_log.h>
#include <string>

#include <esp_system.h>
#include <Esp.h>

/**
 * sample output:
 * ```
 * esp_get_free_heap_size:          4378415
 * esp_get_free_internal_heap_size: 186276
 * esp_get_minimum_free_heap_size:  4373219
 * ESP.getHeapSize:                 306116
 * ESP.getFreeHeap:                 229708
 * ESP.getMinFreeHeap:              224496
 * ESP.getMaxAllocHeap:             110580
 * ```
 */
void test_report_heap() {
  ESP_LOGE(TAG,
    "\nesp_get_free_heap_size: %u"
    "\nesp_get_free_internal_heap_size: %u"
    "\nesp_get_minimum_free_heap_size: %u"
    "\nESP.getHeapSize: %u"
    "\nESP.getFreeHeap: %u"
    "\nESP.getMinFreeHeap: %u"
    "\nESP.getMaxAllocHeap: %u"

    , esp_get_free_heap_size()
    , esp_get_free_internal_heap_size()
    , esp_get_minimum_free_heap_size()
    , ESP.getHeapSize()
    , ESP.getFreeHeap()
    , ESP.getMinFreeHeap()
    , ESP.getMaxAllocHeap()
    );
}

#include <esp_ota_ops.h>
#include <esp_partition.h>

void test_report_partitions() {
  ESP_LOGE(TAG,
    "\npartitions#: %u"
    "\nesp_ota_get_running_partition: %x"
    "\nesp_ota_get_boot_partition: %x"
    "\nesp_ota_get_next_update_partition: %x"
    "\nota0: %x"
    "\nota1: %x"

    , esp_ota_get_app_partition_count()
    , esp_ota_get_running_partition()->address
    , esp_ota_get_boot_partition()->address
    , esp_ota_get_next_update_partition(nullptr)->address
    , esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr)->address
    , esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr)->address
    );
}


/**
 *  NOTE: undef unity.h macros or else:
 *    json.hpp:22020:44: note: in expansion of macro 'isnan'
 *       if ((lhs.is_number_float() && std::isnan(lhs.m_value.number_float) && rhs.is_number())
 *  unity_internals.h:217:18: error: expected unqualified-id before '(' token
 */
#undef isnan
#undef isinf
#include <json.hpp>
#include <NodeInfo.h>

std::string _jdump(const Json &j) {
  /**
   * Without it, bad UTF-8 strings not sanitized while building JSON
   * crash later on `json.dump()`,
   * see https://github.com/nlohmann/json/issues/1198
   */
  constexpr const auto bad_utf8_is_ok = nlohmann::detail::error_handler_t::replace;
  return j.dump(2, ' ', false,  bad_utf8_is_ok);
}
void test_sys_info() {
  node_info_t node_info;
  const auto fixed_info_j = node_info.hw_info_to_json();
  ESP_LOGE(TAG, "HW:\n%s", _jdump(fixed_info_j).c_str());

  const auto status_j = node_info.node_state_to_json();
  ESP_LOGE(TAG, "STATUS:\n%s", _jdump(status_j).c_str());

  const auto info_j = node_info.to_json();
  ESP_LOGE(TAG, "JSON:\n%s", _jdump(info_j).c_str());
}


#define NO_INIT_MAGIC 0x73aec001
__NOINIT_ATTR int boot_counter_magic;
__NOINIT_ATTR int boot_counter;
void test_no_init_ram() {
  delay(3);
  if (boot_counter_magic == NO_INIT_MAGIC) {
    boot_counter++;
  } else {
    boot_counter_magic = NO_INIT_MAGIC;
    boot_counter = 0;
  }
  ESP_LOGE(TAG, "boot_counter_magic: %x, count: %i\n", boot_counter_magic, boot_counter);
  esp_restart();
}


#include <esp_log.h>
#include <esp32-hal-log.h>

inline constexpr const char MY_TAG[] = "MY_TAG";

/**
 * From the sample logs below (with CORE_DEBUG_LEVEL=5 (VERBOSE)),
 * we derrive that must use ESP_LOGx() constructs to have a meaningful "tag"
 * (instead of being fixe to `ARDUINO`).
 *
 * ArduinoCore logs:
 * ```
 * 2-hal-cpu.c:189] setCpuFrequencyMhz(): PLL: 320 / 2 = 160 Mhz, APB: 80000000 Hz
 * [E][test_sandbox.cpp:131] test_logging(): ChipModel: ESP32-D0WDQ6, ChipRevision: 1, SdkVersion: v3.3.5-1-g85c43024c, SketchMD5: bed3b7a145e4c12ddf03fc801120c63e
 * [E][test_sandbox.cpp:132] test_logging(): NO TAG
 * [E][test_sandbox.cpp:133] test_logging(): null TAG
 * [D][test_sandbox.cpp:114] _do_logs(): ESP_LOGV
 * [D][test_sandbox.cpp:114] _do_logs(): ESP_LOGD
 * [I][test_sandbox.cpp:115] _do_logs(): ESP_LOGI
 * [W][test_sandbox.cpp:116] _do_logs(): ESP_LOGW
 * [E][test_sandbox.cpp:117] _do_logs(): ESP_LOGE
 * [D][test_sandbox.cpp:120] _do_logs(): log_d
 * [I][test_sandbox.cpp:121] _do_logs(): log_i
 * [W][test_sandbox.cpp:122] _do_logs(): log_w
 * [E][test_sandbox.cpp:123] _do_logs(): log_e
 * [E][test_sandbox.cpp:138] test_logging(): LEVEL: 5
 * [D][test_sandbox.cpp:114] _do_logs(): ESP_LOGD * ```
 * ```
 *
 * ESP_IDF logs:
 * ```
 * E (48) MyTag: some tag
 * E (49) test_sandbox.cpp: plain
 * E (49) test_sandbox.cpp: ESP_LOGE
 * E (49) test_sandbox.cpp: log_e
 * E (49) test_sandbox.cpp: LEVEL: 5
 * V (52) test_sandbox.cpp: ESP_LOGV
 * D (55) test_sandbox.cpp: ESP_LOGD
 * I (58) test_sandbox.cpp: ESP_LOGI
 * W (61) test_sandbox.cpp: ESP_LOGW
 * E (64) test_sandbox.cpp: ESP_LOGE
 * V (67) test_sandbox.cpp: log_v
 * D (69) test_sandbox.cpp: log_d
 * I (72) test_sandbox.cpp: log_i
 * W (75) test_sandbox.cpp: log_w
 * E (78) test_sandbox.cpp: log_e
 * E (81) test_sandbox.cpp: LEVEL: 4
 * D (84) test_sandbox.cpp: ESP_LOGD
 * I (87) test_sandbox.cpp: ESP_LOGI
 * W (90) test_sandbox.cpp: ESP_LOGW
 * E (93) test_sandbox.cpp: ESP_LOGE
 * D (96) test_sandbox.cpp: log_d
 * I (99) test_sandbox.cpp: log_i
 * W (101) test_sandbox.cpp: log_w
 * E (104) test_sandbox.cpp: log_e
 * E (107) test_sandbox.cpp: LEVEL: 3
 * I (110) test_sandbox.cpp: ESP_LOGI
 * W (113) test_sandbox.cpp: ESP_LOGW
 * E (116) test_sandbox.cpp: ESP_LOGE
 * I (120) test_sandbox.cpp: log_i
 * W (122) test_sandbox.cpp: log_w
 * E (125) test_sandbox.cpp: log_e
 * E (128) test_sandbox.cpp: LEVEL: 2
 * W (131) test_sandbox.cpp: ESP_LOGW
 * E (134) test_sandbox.cpp: ESP_LOGE
 * W (138) test_sandbox.cpp: log_w
 * E (140) test_sandbox.cpp: log_e
 * E (143) test_sandbox.cpp: LEVEL: 1
 * E (146) test_sandbox.cpp: ESP_LOGE
 * E (150) test_sandbox.cpp: log_e
 * E (152) test_sandbox.cpp: LEVEL: 0
 * test_sandbox.cpp:239:test_logging       [PASSED]
 * ```
 */
void _do_logs()
{
    ESP_LOGV(MY_TAG, "ESP_LOGV");
    ESP_LOGD(TAG, "ESP_LOGD");
    ESP_LOGI(TAG, "ESP_LOGI");
    ESP_LOGW(TAG, "ESP_LOGW %s", "foo");
    ESP_LOGE(TAG, "ESP_LOGE");

    log_v("log_v");
    log_d("log_d");
    log_i("log_i");
    log_w("log_w");
    log_e("log_e %s", "foo");
}
void test_logging()
{
    ESP_LOGE("MyTag", "some tag");
    log_e("plain");

    _do_logs();
    for (int i = ARDUHAL_LOG_LEVEL_VERBOSE; i >= ARDUHAL_LOG_LEVEL_NONE; i--)
    {
        ESP_LOGE(TAG, "LEVEL: %i", i);
        esp_log_level_set("*", (esp_log_level_t)i);
        esp_log_level_set(TAG, (esp_log_level_t)i);
        _do_logs();
    }
}


#include <Buzzer.h>
#include <FreematicsPlus.h>  // just for `PIN_BUZZER`

void test_buzzer()
{
    Buzzer buzzer(PIN_BUZZER);

    // volume on VLFs
    constexpr const float lf = 1;
    constexpr const int slow_delay = 1200;
    buzzer.tone(lf, 0.01);
    delay(slow_delay);
    buzzer.tone(lf, 0.2);
    delay(slow_delay);
    buzzer.tone(lf, 0.5);
    delay(slow_delay);
    buzzer.tone(lf, 1.0);
    delay(slow_delay * 2);
    buzzer.tone(lf, 1.5);
    delay(slow_delay);
    buzzer.tone(lf, 1.99);
    delay(slow_delay);
    buzzer.tone(0);

    delay(700);

    float freq = 12;
    buzzer.tone(freq); freq *= 1.618;
    delay(700);
    buzzer.tone(freq); freq *= 1.618;
    delay(700);
    buzzer.tone(freq); freq *= 1.618;
    delay(700);

    ESP_LOGI(TAG, "lower");
    buzzer.tone(1000);
    delay(300);
    buzzer.tone(1000, 8);
    delay(700);

    ESP_LOGI(TAG, "silence");
    buzzer.tone(0);
    delay(300);
    buzzer.tone(0, 0);
    delay(300);
    // buzzer.tone(1000, 0);
    // delay(700);

    buzzer.tone(0);

    ESP_LOGI(TAG, "Logs only");
    Buzzer b2(0);
    b2.tone(freq);      // Should log a warning.
    delay(100);
    b2.tone(freq, 0);   // Should log a warning.
    delay(100);
    b2.tone(0);         // Should log a warning.
}


#include <FreematicsNetwork.h>

ClientWIFI wifi;

uint32_t _check_wifi_status(
    int break_status=WL_CONNECTED, uint32_t timeout_ms=16000, bool notEq=0)
{
    for (uint32_t t = millis(); millis() - t < timeout_ms;) {
        if ((WiFi.status() == break_status) ^ notEq) {
            return millis();
        }
        delay(50);
    }
    return 0;
}

/**
 * Do connect, listAPs, reconnect work when interspersed?
 * Set you WiFi passSSID/pswd in `secrets.ini`
 *
 * ATTENTION: set  valid WIFI_SSID, WIFI_PASSWORD for your network
 * before launching.
 *
 * Ok log sample:
 *
 * ```
 * [I][test_net.cpp:46] test_ClientWIFI_listAPs(): Connecting...
 * [I][test_net.cpp:47] test_ClientWIFI_listAPs(): Connecting...
 * [I][test_net.cpp:51] test_ClientWIFI_listAPs(): Connected after: 5218ms
 * [W][WiFiGeneric.cpp:391] _eventCallback(): Reason: 8 - ASSOC_LEAVE
 * [I][FreematicsNetwork.cpp:83] listAPs(): x4 nearby WiFi APs:
 * [I][FreematicsNetwork.cpp:85] listAPs():   +--1: DIRECT-D3A9B014 (-55s)db)
 * ...
 * [I][test_net.cpp:55] test_ClientWIFI_listAPs(): Reconnecting...
 * [I][test_net.cpp:59] test_ClientWIFI_listAPs(): Reconnected after: 14984ms
 * test/test_net.cpp:64:test_ClientWIFI_listAPs    [PASSED]
 * ```
 */
void test_ClientWIFI_connect_and_listAPs() {
    log_i("Connecting...");
    wifi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t ok_delay_ms = _check_wifi_status();
    // TEST_ASSERT_NOT_EQUAL(0, ok_delay_ms);
    ESP_LOGI("gg", "Connected after: %ims", ok_delay_ms);

    TEST_ASSERT_GREATER_THAN(0, wifi.listAPs());

    log_i("Reconnecting...");
    wifi.reconnect();
    ok_delay_ms = _check_wifi_status();
    TEST_ASSERT_NOT_EQUAL(0, ok_delay_ms);
    log_i("Reconnected after: %ims", ok_delay_ms);

    // ok_delay_ms = _check_wifi_status(WL_CONNECTED, 24000, 1);
    // ESP_LOGI("ss", "Disconnected after: %ims", ok_delay_ms);
}

#include <FS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <fsutil.h>

#include <sstream>

void test_fsutil() {
  File root;
  std::stringstream out;

  out << "SD:\n";
  if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ, "/sd", 5, FORMAT_SD_IF_FAILED)) {
    root = SD.open("/");
    listDir(root, out);
  } else
    ESP_LOGE(TAG, "SD-INIT FAILED!");

  if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    out << "SPIFFS:\n";
    root = SPIFFS.open("/");
    listDir(root, out);
  } else
    ESP_LOGE(TAG, "SPIFFS-INIT FAILED!");

  ESP_LOGE(TAG, "%s", out.str().c_str());
}

# include <multilog.h>
# include <FreematicsPlus.h> // for PIN_SD_CS , SPI_FREQ
#include <FS.h>

void test_SD() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);

    // SPI-num: VSPI
    // #define PIN_NUM_MISO 19
    // #define PIN_NUM_MOSI 23
    // #define PIN_NUM_CLK  18
    // #define PIN_NUM_CS   5
  if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ, "/sd", 5, true)) {
    uint total = SD.totalBytes() >> 20;
    uint used = SD.usedBytes() >> 20;
    ESP_LOGE(TAG, "SD: %i MB total, %i MB used", total, used);

    auto f = SD.open("/ttt", "a");
    ESP_LOGE(TAG, "FOPEN: %i", f.available());
    f.printf("FFF: %li\n", millis());
    ESP_LOGE(TAG, "FWRITE: %i", f.available());
    f.close();
    ESP_LOGE(TAG, "FCLOSED: %i", f.available());

    File root = SD.open("/");
    while(File file = root.openNextFile())
      ESP_LOGE(TAG, "%s: %5u", file.name(), file.size());
    root.close();
  }
}

void process_tests() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    UNITY_BEGIN();
    RUN_TEST(test_sys_info);
    // RUN_TEST(test_report_heap);
    // RUN_TEST(test_report_partitions);
    // RUN_TEST(test_no_init_ram);
    // RUN_TEST(test_logging);
    // RUN_TEST(test_fsutil);
    // RUN_TEST(test_SD);
    // RUN_TEST(test_buzzer);
    // RUN_TEST(test_ClientWIFI_connect_and_listAPs);
    UNITY_END();
}

#ifdef ARDUINO

// #include <Arduino.h>
void setup() {
    // Wait for >2 secs if board doesn't support software reset via Serial.DTR/RTS
    // NOTE: freematics seems to support this.
    // delay(2000);

    process_tests();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}

#else

int main(int argc, char **argv) {
    process_tests();
    return 0;
}

#endif
