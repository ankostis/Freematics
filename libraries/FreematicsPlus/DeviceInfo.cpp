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

#include <esp_log.h>
#include <esp_system.h>
#include <esp_app_format.h>
#include <Esp.h>
#include <esp_ota_ops.h>
#include <soc/rtc.h>
#if BOARD_HAS_PSRAM && BOARD_HAS_PSRAM_HIGH
#   include "esp32/himem.h"
#endif
#include "DeviceInfo.h"

void LogDeviceInfo(const char *devid)
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
    ESP_LOGE(TAG, "device_id: %s"
        "\n+--     board: %s-v%i@%ix%i"
        ",\n+--       mac: %llx"
        ",\n+--     flash: %iMiB@%iMHz"
        ",\n+--  slow_rtc: %i@%iKHz"
        ",\n+-- esp32_sdk: %s"
        ",\n+--   arduino: %i.%i.%i"
        ",\n+--fw_version: %s"
        ",\n+--sketch_elf: %.7s-%lx" // elf_sha256 is 32-bytes actually (not just 4)
        ",\n+--     build: %s (%s@%s)"
        ",\n+--last_boot: %i"
        ",\n+-- partition: %6.2f%%, sketch_size: %7i, partition_size: %7i"
        ",\n+--      heap: %6.2f%%,   heap_used: %7i,      heap_size: %7i"
#if BOARD_HAS_PSRAM
        ",\n+--     psram: %6.2f%%,  psram_used: %7i,     psram_size: %5.2f%%MiB"
#   if BOARD_HAS_PSRAM_HIGH
        ",\n+--psram_high: %6.2f%%, psram_high_used: %7i, psram_high_size: %5.2f%%MiB"
#   endif
#endif
        "\n"
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
#   endif  // BOARD_HAS_PSRAM_HIGH
#endif  // BOARD_HAS_PSRAM
        );
#pragma GCC diagnostic pop

#if BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
    ESP_LOGI(TAG, "Writing PSRAM to validate it...");
    uint32_t *ptr = (uint32_t*)ps_malloc(PsramSize);
    if (!ptr) {
      ESP_LOGE(TAG, "unable to allocate %i bytes", PsramSize);
    } else {
      uint32_t t = millis();
      for (int i = 0; i < PsramSize / 4; i++) {
        ptr[i] = 0xa5a5a5a5;
      }
      ESP_LOGI(TAG, "OK @%iKB/s", PsramSize  / (millis() - t));
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
      ESP_LOGI(TAG, "OK @%iKB/s", PsramSize  / (millis() - t));
    }
    free(ptr);
#endif // BOARD_HAS_PSRAM && PSRAM_VALIDATE_CAN_WRITE
}
