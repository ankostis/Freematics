/**
 * Deflect logs (in addition to Serial) also to SD or SPIFFS-memory.
 */
#pragma once

#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

namespace logsinks {

constexpr const char *default_log_fpath = "/logs.txt";
constexpr const float default_disk_usage_purge_ratio = 0.9f;
constexpr const time_t default_log_sink_sync_interval_ms = 3141;

struct LogSink {
  LogSink(const char *name, FS fs, const char *fpath, const char *open_mode,
          float disk_usage_purge_ratio, time_t sync_interval_ms)
      : name{name},
        fs{fs},
        fpath{fpath},
        open_mode{open_mode},
        disk_usage_purge_ratio{disk_usage_purge_ratio},
        sync_interval_ms(sync_interval_ms),
        last_synced_ms(0) {}
  /**
   * When `enable` true, start writting logs also to SD-card/SPIFFS.
   * (needs `DUSE_ESP_IDF_LOG=1` in build-flags.)
   *
   * NOTE: SD & SPIFFS MUST have been initialized before enabling stuff here!
   */
  void enable(BaseType_t enabled);
  void flush();
  float disk_usage_ratio();
  void close() { enable(false); }
  BaseType_t enabled() { return file.available(); }
  void purge() {
    if (enabled()) enable(true);
  }

  //  protected:  // No, we're adults.
  const char *name;
  FS &fs;
  const char *fpath;
  const char *open_mode;
  /** When to delete log-file?  Check happens when enabling. */
  float disk_usage_purge_ratio;
  /** How often to sync log-file?  Check happens when writing logs. */
  time_t sync_interval_ms;
  time_t last_synced_ms;
  File file;
};

/** public, for adults to see. */
inline LogSink log_sinks[]{
    {
        "SD",
        SD,  // FIXME: it's not equal to "public" fs instance!
        default_log_fpath,
        FILE_APPEND,
        default_disk_usage_purge_ratio,
        default_log_sink_sync_interval_ms,
    },
    {
        "SPIFFS",
        SPIFFS,  // FIXME: it's not equal to "public" fs instance!
        default_log_fpath,
        FILE_WRITE,
        default_disk_usage_purge_ratio,
        default_log_sink_sync_interval_ms,
    },
};

}  // namespace logsinks