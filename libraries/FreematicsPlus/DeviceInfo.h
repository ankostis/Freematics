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
#include "FreematicsPlus.h"
/**
 * Sample output:
 *
 * ```
 * device_id: ABCDFGZRJ
 * +--     board: ESP32-D0WDQ6-v1@160x2,
 * +--       mac: f4f78bc40a24,
 * +--     flash: 16MiB@40MHz,
 * +--  slow_rtc: 0@150KHz,
 * +-- esp32_sdk: v4.4-367-gc29343eb94,
 * +--   arduino: 2.0.3,
 * +--fw_version: v0.1.0-upstream202204-27-g5b707fd,
 * +--sketch_elf: 183ec6f-3f4000b0,
 * +--     build: Apr 13 2022 13:37:40 (user@host),
 * +--last_boot: 1,
 * +-- partition:  20.03%, sketch_size:  262528, partition_size: 1310720,
 * +--      heap:   6.84%,   heap_used:   25248,      heap_size:  369160
 * ```
 *
 * ATTENTION: rev1 devices (like the sample above),
 * [need the PSRAM workaround](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/external-ram.html#esp32-rev-v1)
 *
 */
void LogDeviceInfo(const freematics_cfg_t &node_cfg);
