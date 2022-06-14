/**
 * Deflect logs (in addition to Serial) also to SD or SPIFFS-memory.
 */
#include <FS.h>
#include <multilog.h>

using namespace fs;
using namespace multilog;

//////////////////////////
//// IDF-LOG replacement machinery
//////////////////////////
/**
 * This fn replaces ISP-IDF's logs receiver (when enabled).
 *
 * Build final msg with `vsnprintf()` ang call directly `file/Serial.write()`,
 * (instead of replicating formatting inside both `v(f)printf()`).
 *
 * Adapted from:
 *      arduino-esp32.git/cores/esp32/esp32-hal-uart.c::log_printf()
 * ... after consulting:
 *      esp-idf.git/components/log/log.c::esp_log_writev()
 */
static int _log_printf(const char *format, va_list args) {
  char stack_buf[MULTILOG_INITIAL_SPRINTF_BUFLEN];
  char *buf = stack_buf;
  int len, buflen = sizeof(stack_buf);

  while ((len = vsnprintf(buf, buflen, format, args)) >= buflen)
    if (len < 0 || !(buf = (char *)malloc(buflen = len + 1))) return 0;

  for (auto sink : log_sinks)
    if (sink) sink->write(buf, len);

  if (buf != stack_buf) free(buf);

  return len;
}

//////////////////////////
//// Public API
//////////////////////////

bool FileSink::enableChanged(bool enabled) {
  bool exist = false, purge = false;
  // Disable does nothing and always succeeds.
  bool ok = !enabled;

  if (enabled) {
    exist = fs.exists(fpath);
    purge = exist && disk_usage_ratio() >= disk_usage_purge_ratio;
    if (purge) fs.remove(fpath);

    file = fs.open(fpath, open_mode);

    // NOTE: write something before checking `file.available()`, below
    // or else, it returns 0 immediately after `open()`!
    int len = file.printf("--((NEW SESSION LOG@%lu))--\n", millis());
    ok = file.available() || len;
    last_synced_ms = millis();
  }

  ESP_LOGI(TAG_MULTILOG, "%sable multi-logs --> file(%s)%s%s: %s",
           (enabled ? "En" : "Dis"), name.c_str(),
           (exist ? " which existed already" : ""),
           (purge ? " and was purged" : ""), (ok ? "ok" : "FAILED!"));

  return ok;
}

void FileSink::write(const char *buf, int buflen) {
  if (!file) file = fs.open(fpath, open_mode);
  file.write((uint8_t *)buf, buflen);
  time_t now = millis();
  if ((now - last_synced_ms) > sync_interval_ms) {
       file.flush();
       last_synced_ms = now;
  }
}

float FileSink::disk_usage_ratio() {
  // AARGH, no virtual!
  const bool is_sd = name == "SD";
  float usedBytes = is_sd ? SD.usedBytes() : SPIFFS.usedBytes();
  float totalBytes = is_sd ? SD.totalBytes() : SPIFFS.totalBytes();
  return (double)usedBytes / totalBytes;
}

void multilog::enable(bool enable) {
  esp_log_set_vprintf(enable ? &_log_printf : &vprintf);

  for (auto sink : log_sinks)
    if (sink) sink->enableChanged(enable);
}
