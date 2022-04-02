
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

#include <Esp.h>
#include <esp_ota_ops.h>
#if BOARD_HAS_PSRAM && BOARD_HAS_PSRAM_HIGH
#   include <himem.h>
#endif
#include <soc/rtc.h>

/**
 *
 * ATTENTION: rev1 devices (like the sample below),
 * [need the PSRAM workaround](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html#esp32-rev-v1)
 *
 * Sample output:
 * ```
 * SysInfo:
 *        cpu: 160MHz, chip_model: ESP32-D0WDQ6-v1,
 * flash_size: 16MiB, flash_speed: 40MHz, slow_rtc_freq: 150.0KHz,
 *  esp32_sdk: v4.4-367-gc29343eb94,
 *    arduino: 2.0.3,
 * fw_version: v0.1.0-upstream202204-14-g798d4ef,
 * sketch_md5: 6ff4796,
 * build_date: Apr  7 2022 13:23:16,
 *  partition:  20.70%, sketch_size:  271312, partition_size: 1310720,
 *       heap:   6.94%,   heap_used:  339236,      heap_size:  364532,
 *      psram:   0.00%,  psram_used: 4192139,     psram_size:  4.00MiB
 * ```
 */
void test_sys_info()
{
    int PartitionSize = esp_ota_get_running_partition()->size;  // +395 from compiler report
    int SketchSize = ESP.getSketchSize();

    int HeapSize = ESP.getHeapSize();
    int HeapFree = ESP.getFreeHeap();
    int HeapUsed = HeapSize - HeapFree;

#if BOARD_HAS_PSRAM
    int PsramSize = ESP.getPsramSize();
    int PsramFree = ESP.getFreePsram();
    int PsramUsed =  PsramSize - PsramFree ;

#   if BOARD_HAS_PSRAM_HIGH
    int PsramHighSize = (int)esp_himem_get_phys_size();
    int PsramHighFree = (int)esp_himem_get_free_size();
    int PsramHighUsed = PsramHighSize - PsramHighFree;
#   endif
#endif

    Serial.printf(
        "SysInfo"
        ":\n       cpu: %iMHz, chip_model: %s-v%i"
        ",\nflash_size: %iMiB, flash_speed: %.2fMHz, slow_rtc: %i@%.2fKHz"
        ",\n esp32_sdk: %s"
        ",\n   arduino: %i.%i.%i"
        ",\nfw_version: %s"
        ",\nsketch_md5: %.7s"
        ",\nbuild_date: %s"
        ",\n partition: %6.2f%%, sketch_size: %7i, partition_size: %7i"
        ",\n      heap: %6.2f%%,   heap_used: %7i,      heap_size: %7i"
#if BOARD_HAS_PSRAM
        ",\n     psram: %6.2f%%,  psram_used: %7i,     psram_size: %5.2f%%MiB"
#   if BOARD_HAS_PSRAM_HIGH
        ",\npsram_high: %6.2f%%, psram_high_used: %7i, psram_high_size: %5.2f%%MiB"
#   endif
#endif
        "\n"
        , ESP.getCpuFreqMHz()
        , ESP.getChipModel()
        , ESP.getChipRevision()
        , ESP.getFlashChipSize() >> 20
        , (float)ESP.getFlashChipSpeed() / 1000000
        , rtc_clk_slow_freq_get()
        , (float)rtc_clk_slow_freq_get_hz() / 1000
        , IDF_VER
        , ESP_ARDUINO_VERSION_MAJOR
        , ESP_ARDUINO_VERSION_MINOR
        , ESP_ARDUINO_VERSION_PATCH
        , GIT_DESCRIBE
        , ESP.getSketchMD5().c_str()
        , __DATE__ " " __TIME__
        , 100.0 * ((float)SketchSize / PartitionSize)
        , SketchSize
        , PartitionSize
        // , NOTE: `ESP.getFreeSketchSpace()` actually brings next (OTA) partion's siz
        // , (see espressif/arduino-esp32#3501
        // , ESP.getFreeSketchSpace()
        , 100.0 * ((float)HeapUsed / HeapSize)
        , HeapUsed
        , HeapSize
#if BOARD_HAS_PSRAM
        , 100.0 * ((float)PsramUsed / PsramSize)
        , PsramUsed
        , (float)PsramSize / (1<<20)  // not exactly 4MiB reported
#   if BOARD_HAS_PSRAM_HIGH
        , 100.0 * ((float)PsramHighUsed / PsramHighSize)
        , PsramHighUsed
        , (float)PsramHighSize / (1<<20)
#   endif
#endif
        );
}

#include "esp_log.h"
#include "esp32-hal-log.h"

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
    ESP_LOGV(TAG, "ESP_LOGV");
    ESP_LOGD(TAG, "ESP_LOGD");
    ESP_LOGI(TAG, "ESP_LOGI");
    ESP_LOGW(TAG, "ESP_LOGW");
    ESP_LOGE(TAG, "ESP_LOGE");

    log_v("log_v");
    log_d("log_d");
    log_i("log_i");
    log_w("log_w");
    log_e("log_e");
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

#include "FreematicsNetwork.h"

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


void process() {
    UNITY_BEGIN();
    RUN_TEST(test_sys_info);
    RUN_TEST(test_logging);
    RUN_TEST(test_ClientWIFI_connect_and_listAPs);
    UNITY_END();
}

#ifdef ARDUINO

// #include <Arduino.h>
void setup() {
    // Wait for >2 secs if board doesn't support software reset via Serial.DTR/RTS
    // NOTE: freematics seems to support this.
    // delay(2000);

    process();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}

#else

int main(int argc, char **argv) {
    process();
    return 0;
}

#endif
