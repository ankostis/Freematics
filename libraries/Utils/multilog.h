/**
 * Deflect ESP_IDF logs (in addition to Serial) also to SD or SPIFFS-memory.
 */
#pragma once

#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

#include <string>

namespace multilog {

constexpr const int MULTILOG_INITIAL_SPRINTF_BUFLEN = 256;
constexpr const char TAG_MULTILOG[] = "ENABLE_MULTILOG";

struct Sink {
 public:
  Sink(std::string name) : name{name} {};
  virtual bool enableChanged(bool enabled) {
    ESP_LOGI(TAG_MULTILOG, "%sabled multi-logs --> %s.",
             (enabled ? "En" : "Dis"), name.c_str());
    return true;
  }
  virtual void write(const char *buf, int buflen) = 0;
  /** debugging aid */
  std::string name;
};

struct SerialSink : public Sink {
 public:
  SerialSink() : Sink("Serial"){};
  /** Called after `lock()` - `unlock()`. */
  virtual void write(const char *buf, int buflen) { Serial.write(buf, buflen); }
};

struct FileSink : public Sink {
 public:
  /*
   * Start writing logs also to SD-card/SPIFFS.
   * (needs `DUSE_ESP_IDF_LOG=1` in build-flags.)
   *
   * NOTE: SD & SPIFFS MUST have been initialized before enabling stuff here!
   */
  FileSink(std::string name, FS &fs, const char *fpath, const char *open_mode,
           float disk_usage_purge_ratio)
      : Sink(name + ":" + fpath),
        fs{fs},
        fpath{fpath},
        open_mode{open_mode},
        disk_usage_purge_ratio{disk_usage_purge_ratio} {};
  virtual bool enableChanged(bool enabled);
  virtual void write(const char *buf, int buflen);
  float disk_usage_ratio();

  //  protected:  // No, we're adults.
  FS &fs;
  const char *fpath;
  const char *open_mode;
  /** When to delete log-file?  Check happens when enabling. */
  float disk_usage_purge_ratio;
};

/**
 * Surround  operations on log-sinks with `lock()`/`unlock()` calls.
 *
 * ATTENTION: do not issue log-statements when holding lock!
 *
 * Copied from esp-idf/components/log/log_freertos.c
 */
bool lock();
void unlock();

/**
 * ATTENTION: guard any operation on this list with `lock()`/`unlock()` calls.
 * NOTE: take care the memory of items inserted & deleted.
 *
 * Public, for adults to see.
 */
inline Sink *log_sinks[3]{nullptr};

/**
 * :param enable:
 *    When `true`, logs reach multiple log-sinks (or none if empty), otherwise
 *    the default ESP_IDF logging-behavior is restored and existing loggers
 *    are closed.
 *
 * :see-also: `esp_log.h:esp_log_set_vprintf()`.
 */
void enable(bool enable);

}  // namespace multilog