
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

#include <config.h>
#include <FreematicsNetwork.h>
#include <unity.h>
// #include "esp32-hal-log.h"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

/**
 * From the sample logs below,
 * we derrive that must use ESP_LOGx() constructs to have a meaningful "tag"
 * (instead of being fixe to `ARDUINO`).
 *
 * ArduinoCore logs:
 * ```
 * 2-hal-cpu.c:189] setCpuFrequencyMhz(): PLL: 320 / 2 = 160 Mhz, APB: 80000000 Hz
 * [E][test_sandbox.cpp:131] test_logging(): ChipModel: ESP32-D0WDQ6, ChipRevision: 1, SdkVersion: v3.3.5-1-g85c43024c, SketchMD5: bed3b7a145e4c12ddf03fc801120c63e
 * [E][test_sandbox.cpp:132] test_logging(): NO TAG
 * [E][test_sandbox.cpp:133] test_logging(): null TAG
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
 * PLL: 320 / 2 = 160 Mhz, APB: 80000000 Hz
 * E (186) TAG: ChipModel: ESP32-D0WDQ6, ChipRevision: 1, SdkVersion: v3.3.5-1-g85c43024c, SketchMD5: 3f878b6faf761f5b14fd71b3ad2dd719
 * E (451) : NO TAG
 * E (452) TAG: ESP_LOGE
 * E (454) ARDUINO: log_e
 * E (456) ARDUINO: LEVEL: 5
 * D (459) TAG: ESP_LOGD
 * I (461) TAG: ESP_LOGI
 * W (463) TAG: ESP_LOGW
 * E (465) TAG: ESP_LOGE
 * D (467) ARDUINO: log_d
 * I (469) ARDUINO: log_i
 * W (471) ARDUINO: log_w
 * E (473) ARDUINO: log_e
 * E (475) ARDUINO: LEVEL: 4
 * D (477) TAG: ESP_LOGD
 * I (479) TAG: ESP_LOGI
 * W (481) TAG: ESP_LOGW
 * E (483) TAG: ESP_LOGE
 * D (485) ARDUINO: log_d
 * I (487) ARDUINO: log_i
 * W (490) ARDUINO: log_w
 * E (492) ARDUINO: log_e
 * E (494) ARDUINO: LEVEL: 3
 * I (496) TAG: ESP_LOGI
 * W (498) TAG: ESP_LOGW
 * E (500) TAG: ESP_LOGE
 * I (502) ARDUINO: log_i
 * W (504) ARDUINO: log_w
 * E (506) ARDUINO: log_e
 * E (508) ARDUINO: LEVEL: 2
 * W (511) TAG: ESP_LOGW
 * E (513) TAG: ESP_LOGE
 * W (515) ARDUINO: log_w
 * E (517) ARDUINO: log_e
 * E (519) ARDUINO: LEVEL: 1
 * E (521) TAG: ESP_LOGE
 * E (523) ARDUINO: log_e
 * E (525) ARDUINO: LEVEL: 0
 * ```
 */
void _do_logs()
{
    ESP_LOGV("MyTAG", "ESP_LOGV");
    ESP_LOGD("MyTAG", "ESP_LOGD");
    ESP_LOGI("MyTAG", "ESP_LOGI");
    ESP_LOGW("MyTAG", "ESP_LOGW");
    ESP_LOGE("MyTAG", "ESP_LOGE");

    log_v("log_v");
    log_d("log_d");
    log_i("log_i");
    log_w("log_w");
    log_e("log_e");
}
void test_logging()
{
    ESP_LOGE(
        "MyTAG",
        "SysInfo:\n+--ChipModel: %s,\n+--ChipRevision: %i,"
        "\n+--SdkVersion: %s,  \n+--SketchMD5: %s,\n+--CommitId: %s"
        "\n+--FlashChipSize: %i(%i),  \n+--getHeapSize: %i(%i),\n+--PsramSize: %i(%i)",
        ESP.getChipModel(),
        ESP.getChipRevision(),
        ESP.getSdkVersion(),
        ESP.getSketchMD5().c_str(),
        GIT_DESCRIBE,
        ESP.getFlashChipSize(),
        ESP.getFreeSketchSpace(),
        ESP.getHeapSize(),
        ESP.getHeapSize(),
        ESP.getPsramSize(),
        ESP.getFreePsram()
    );
    ESP_LOGE("MyTAG", "NO TAG");
    log_e("plain");

    _do_logs();
    for (int i = ARDUHAL_LOG_LEVEL_VERBOSE+1; i >= ARDUHAL_LOG_LEVEL_NONE; i--)
    {
        ESP_LOGE("MyTAG", "LEVEL: %i", i);
        esp_log_level_set("*", (esp_log_level_t)i);
        esp_log_level_set("MyTAG", (esp_log_level_t)i);
        _do_logs();
    }
}


void process() {
    UNITY_BEGIN();
    RUN_TEST(test_logging);
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
