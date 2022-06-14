#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Circular Buffer Configuration
**************************************/
/**
 * Max number of buffers
 * If limit reached, the oldest slot is purged and re-populated,
 * hence, gaps & out-of-order rows appear in the trace.
 */
#define BUFFER_SLOTS 256
/**
 * Bytes-per-slot
 * PID-samples not fitting in the current slot, they are dropped!
 */
#define BUFFER_LENGTH 180
#define SERIALIZE_BUFFER_SIZE 1024 /* bytes */

/**************************************
* Configuration Definitions
**************************************/
#define NET_WIFI 1
#define NET_SIM800 2
#define NET_SIM5360 3
#define NET_SIM7600 4
#define NET_WIFI_MESH 5
#define NET_SERIAL 6

#define LOG_SINK_NONE   0
#define LOG_SINK_SERIAL 0x1
#define LOG_SINK_SD     0x2
/**
 * NOTE: Logging into SPIFFS is not a good idea, prefer to persist them into SD.
 * SPIFFS has reduced space and may fragment, suited for persisting few
 * controlled data-files.
 * Besides, the docs mention over a second write-time, occasionally:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spiffs.html#notes
 */
#define LOG_SINK_SPIFFS 0x4

#define STORAGE_NONE    0
/**
 * Store trip-traces into SD/SPIFFS;  SD & SPIFFS are mutual-exclusive.
 * NOTE: Storing trips delays boot due to enumerating old files.
 */
#define STORAGE_SPIFFS  1
#define STORAGE_SD      2

#define GNSS_NONE 0
#define GNSS_STANDALONE 1
#define GNSS_CELLULAR 2

#define PROTOCOL_UDP 1
#define PROTOCOL_HTTPS 2

#define PROTOCOL_METHOD_GET 0
#define PROTOCOL_METHOD_POST 1

#define LOG_EXT_SENSORS_NONE    0
#define LOG_EXT_SENSORS_DIGITAL 1
#define LOG_EXT_SENSORS_ANALOG  2

/**************************************
 * Logging (see also `platformio.ini`)
 **************************************/
// Works only when ESP_IDF logging-lib selected in `platformio.ini`.
#define RUNTIME_ALL_TAGS_LOG_LEVEL CORE_DEBUG_LEVEL

/**
 * Whether to enabled multiple log destinations (sinks).
 *
 * NOTE: enabling this without any `LOG_SINK_XX` will produce no logs at all!
 */
#define ENABLE_MULTILOG                     0
/**
 * Which log destinations (sinks) to enable (relevant only if `ENABLE_MULTILOG`).
 * Either `LOG_SINK_NONE` or `LOG_SINK_XXX` constants OR-ed together.
 */
#define LOG_SINK                            LOG_SINK_SERIAL
#define LOG_SINK_FPATH                      "/logs.txt"
#define LOG_SINK_DISK_USAGE_PURGE_RATIO     0.90f
#define LOG_SINK_SYNC_INTERVAL_MS           3141

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif

// maximum consecutive OBD access errors before entering standby
#define MAX_OBD_ERRORS 3

/**************************************
 * Networking configurations
 **************************************
 * Don't modify per-device network settings & secrets here,
 * do it in `secrets.h` overrides instead:
 *      #define NET_DEVICE NET_xxx   //(default below)
 *      #define WIFI_SSID ""
 *      #define WIFI_SSID ""
 *      #define CELL_APN ""
 *      #define SIM_CARD_PIN ""
 *      #define SERVER_HOST "hub.freematics.com"
 */
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_WIFI
// WiFi settings
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"
// cellular network settings
#define CELL_APN ""
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PROTOCOL PROTOCOL_UDP
#endif

// SIM card setting
#define SIM_CARD_PIN ""

// HTTPS settings
#define SERVER_METHOD PROTOCOL_METHOD_POST
#define SERVER_PATH "/hub/api"

#if !SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT 8081
#elif SERVER_PROTOCOL == PROTOCOL_HTTPS
#define SERVER_PORT 443
#endif
#endif

// WiFi Mesh settings
#define WIFI_MESH_ID "123456"
#define WIFI_MESH_CHANNEL 13

// WiFi AP settings
#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// Currently, only server's URL * WiFi passwords considered secrets.
#define HIDE_SECRETS_IN_LOGS 0

// Attempts to open net-connection before reporting error.
#define NET_CONNECT_RETRIES 5
// How many time to attempt opening net-connection
#define UDP_CONNECT_RETRY_DELAY_MS  3000
// maximum consecutive communication errors before resetting network
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */  // TODO: drop unused cfg
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// data interval settings
#define STATIONARY_TIME_TABLE {20, 40, 60} /* seconds */
#define DATA_INTERVAL_TABLE {1000, 2000, 5000} /* ms */
#define PING_BACK_INTERVAL 900 /* seconds */

// How often to send PIDs form the on-board fuel-consumption monitoring device.
#define OBFCM_INTERVAL_MS 120000

/**************************************
* Data storage configurations
**************************************/
#ifndef STORAGE
// change the following line to change storage type
#define STORAGE STORAGE_NONE
#endif

/**************************************
* MEMS sensors
**************************************/
#define ENABLE_ORIENTATION 0
#ifndef ENABLE_MEMS
#define ENABLE_MEMS 1
#endif

/**************************************
* GPS
**************************************/
#ifndef GNSS
// change the following line to change GNSS setting
#define GNSS GNSS_STANDALONE
#endif
#define GPS_SERIAL_BAUDRATE 115200L
#define GPS_MOTION_TIMEOUT 180 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
// reset device after waking up
#define RESET_AFTER_WAKEUP 0
// motion threshold for waking up
#define MOTION_THRESHOLD 0.4f /* moving vehicle motion threshold in G */
// engine jumpstart voltage for waking up (when ENABLE_MEMS)
#define THR_VOLTAGE 13.6 /* V */
// engine jumpstart voltage gradient
#define THR_GRAD 1 /* V */

/**************************************
* Additional features
**************************************/
/**
 * Enable filesystem access commands?
 * The respective filesystem is implicitly enabled when
 * any of `STORAGE_XXX` or `MULTILINE+LOG_SINK_XXX` are used.
 */
#define ENABLE_SD       0
#define ENABLE_SPIFFS   0
#define FORMAT_SD_IF_FAILED                 true
#define FORMAT_SPIFFS_IF_FAILED             true

/** How long commands read from the serial can be? */
#define CMD_SERIAL_MAX_LEN     128
/** How many lines `HEAD` command prints? */
#define CMD_HEAD_NLINES        64
/** How many bytes the `TAIL` command to backtrack from the end-of-file? */
#define CMD_TAIL_NBYTES        -4096

/**
 * Over-the-air firmware-upgrade from HTTPS enabled?
 * Performed with "OTA[ url]" command,
 * where any `path` is appended at the end of the `OTA_UPDATE_URL`
 *
 * If enabled, MUST also define `OTA_UPDATE_URL` and `OTA_UPDATE_CERT_PEM`.
 */
#define ENABLE_OTA_UPDATE      0
/** The HTTPS site to download the firmware from. */
#define OTA_UPDATE_URL  ""
/**
 * The certificate-chain in pem format is needed here,
 * taken from, eg `/etc/letsencrypt/live/<server.url>/chain.pem`.
 */
#define OTA_UPDATE_CERT_PEM  ""

// When enabled, significant states produce buzzing patterns,
// as explained in `./README.md` file.
#define ENABLE_BUZZING_INIT 1
/**
 * Enable(1)/disable(0) http server for `dataserver.cpp`.
 * (httpd has been dropped from `telelogger.ino`).
 *
 * NOTE: MUST enable also some STORAGE_SD/SPIFFS, for having something to send,
 * or else compilation fails with:
 *      dataserver.cpp:243:9: error: 'root' was not declared in this scope
 */
#define ENABLE_HTTPD 0
// enable(1)/disable(0) OLED_SH1106 screen (if connected to the board).
#define ENABLE_OLED 0
#define CONFIG_MODE_TIMEOUT 0

#define PIN_SENSOR1 34
#define PIN_SENSOR2 26

#define LOG_EXT_SENSORS LOG_EXT_SENSORS_NONE

#define COOLING_DOWN_TEMP 80 /* celsius degrees */
#define COOLING_DOWN_SLEEP_SEC 5 /* celsius degrees */

/**************************************
 * Secrets & per-device overrides, like:
 * ```
 * // git-ignored header-file for "sensitive" or per-device overrides.
 *
 * #undef NET_DEVICE
 *
 * #define NET_DEVICE NET_XXX
 *
 * #undef WIFI_SSID
 * #define WIFI_SSID "..."
 * #undef WIFI_PASSWORD
 * #define WIFI_PASSWORD "..."
 *
 * #undef CELL_APN
 * #define CELL_APN "..."
 * #undef SIM_CARD_PIN
 * #define SIM_CARD_PIN "..."
 *
 * #undef SERVER_HOST
 * #define SERVER_HOST "..."
 * ```
 **************************************/
#if __has_include("secrets.h")
#   include "secrets.h"
#endif

#define _NEED_SD        (ENABLE_SD || (STORAGE == STORAGE_SD) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG && (LOG_SINK & LOG_SINK_SD)))
#define _NEED_SPIFFS    (ENABLE_SPIFFS || (STORAGE == STORAGE_SPIFFS) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG && (LOG_SINK & LOG_SINK_SPIFFS)))

#endif // CONFIG_H_INCLUDED
