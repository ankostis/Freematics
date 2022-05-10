/**
 * Deflect logs (in addition to Serial) also to SD or SPIFFS-memory.
 */
#include <FS.h>
#include <LogSink.h>
#include <SD.h>
#include <SPIFFS.h>
#include <stdio.h>

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

using namespace fs;
using namespace logsinks;

//////////////////////////
//// IDF-LOG replacement machinery
//////////////////////////

// Maximum time to wait for the mutex in a logging statement.
// (see `log_freertos.c`)
#define MAX_MUTEX_WAIT_MS 10
#define MAX_MUTEX_WAIT_TICKS \
  ((MAX_MUTEX_WAIT_MS + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

static SemaphoreHandle_t s_mutex = nullptr;

/**
 * To protect operations on the log-files.
 *
 * Copied from esp-idf/components/log/log_freertos.c
 */
static BaseType_t _lock() {
  if (unlikely(!s_mutex)) {
    s_mutex = xSemaphoreCreateMutex();
  }
  if (unlikely(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)) {
    return true;
  }
  return xSemaphoreTake(s_mutex, MAX_MUTEX_WAIT_TICKS) == pdTRUE;
}
void _unlock(void) {
  if (unlikely(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)) {
    return;
  }
  xSemaphoreGive(s_mutex);
}

/**
 * This fn replaces ISP-IDF's logs receiver (when enabled).
 *
 * Build final msg with `vsnprintf()` ang call directly `file/Serial.write()`,
 * (instead of replicating formatting inside both `v(f)printf()`).
 */
static int _log_printf(const char *format, va_list args) {
  char loc_buf[256];
  char *temp = loc_buf;
  int len = vsnprintf(temp, sizeof(loc_buf), format, args);
  if (len < 0) {
    return 0;
  };
  if (len >= sizeof(loc_buf)) {
    temp = (char *)malloc(len + 1);
    if (temp == NULL) {
      return 0;
    }
    len = vsnprintf(temp, len + 1, format, args);
  }

  int len1, len2;

  len1 = Serial.write(temp, len);  // TODO: write serial-logs in an RTOS-task.

  _lock();

  time_t now = millis();

  for (auto &sink : log_sinks) {
    if (sink.enabled()) {
      // TODO: check indeed written, not just the last one.
      len2 = sink.file.write((uint8_t *)temp, len);

      if ((now - sink.last_synced_ms) > sink.sync_interval_ms) {
        sink.file.flush();
        sink.last_synced_ms = now;
      }
    }
  }

  _unlock();

  if (temp != loc_buf) {
    free(temp);
  }
  return (len == len1) && (len == len2) ? len : 0;
}

//////////////////////////
//// Public API
//////////////////////////

BaseType_t log_sinks_any_enabled() {
  for (auto &sink : log_sinks)
    if (sink.enabled()) return true;
  return false;
}

/**
 * Adapted from:
 *      arduino-esp32.git/cores/esp32/esp32-hal-uart.c::log_printf()
 * ... after consulting:
 *      esp-idf.git/components/log/log.c::esp_log_writev()
 */
void LogSink::enable(BaseType_t enabled) {
  ESP_LOGI(TAG, "Logs --> file(%s:%s): %u -> %u", name, fpath, file.available(),
           (fpath != nullptr));
  _lock();

  file.close();  // flush any pending logs.

  if (enabled) {
    if (disk_usage_ratio() >= disk_usage_purge_ratio) {
      fs.remove(fpath);
    }

    File log_file = fs.open(fpath, open_mode);
    // NOTE: write something before checking `file.available()`, below
    // or else, it returns 0 immediately after `open()`!
    uint32_t fsize = log_file.size();
    if (fsize)
      ESP_LOGI(TAG, "Found %u bytes in old log-file(%s)", fsize, fpath);
    log_file.printf("--((NEW SESSION LOG@%lu))--\n", millis());
    if (log_file.available()) {
      file = log_file;
    } else {
      ESP_LOGE(TAG, "Failed opening log-file(%s:%s)!", name, fpath);
    }
  } else {  // (!enabled)?
    file.close();
  }
  esp_log_set_vprintf(log_sinks_any_enabled() ? &_log_printf : &vprintf);

  _unlock();
}

float LogSink::disk_usage_ratio() {
  // AARGH, no virtual!
  const BaseType_t is_sd = strcmp(name, "SD");
  float usedBytes = is_sd ? SD.usedBytes() : SPIFFS.usedBytes();
  float totalBytes = is_sd ? SD.totalBytes() : SPIFFS.totalBytes();
  return (double)usedBytes / totalBytes;
}

void LogSink::flush() {
  _lock();
  file.flush();
  _unlock();
}
