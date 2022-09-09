/**
 * Configuration macros for `src_dir` code AND libs.
 *
 * TIP: Whichever config option below is marked with **json-config default(s)**,
 * it can change on *runtime*.
 *
 * NOTE: Instead of putting secrets & per device/user settings here
 * (and git-committing them!)
 * prefer to store them in an out-of-git file `secrets.h`, like this:
 *
 * ```
 * // git-ignored header-file for "sensitive" or per-device/user overrides.
 *
 * #undef ENABLE_MULTILOG
 * #define ENABLE_MULTILOG    0
 * #undef LOG_SINK
 * #define LOG_SINK    (LOG_SINK_SERIAL| LOG_SINK_SD)

 * #undef NET_DEVICE
 *
 * #define NET_DEVICE NET_XXX
 *
 * #undef WIFI_SSIDS
 * #define WIFI_SSIDS {"ssid1", "pswd1"}, {"ssid2", "pswd2"}
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
#define BUFFER_SLOTS            256
/**
 * Bytes-per-slot
 * PID-samples not fitting in the current slot, they are dropped!
 */
#define BUFFER_LENGTH           180
#define SERIALIZE_BUFFER_SIZE   1024 /* bytes */

/**************************************
* Configuration Definitions
**************************************/
#define NET_WIFI                1
#define NET_SIM800              2
#define NET_SIM5360             3
#define NET_SIM7600             4
#define NET_WIFI_MESH           5
#define NET_SERIAL              6

#define LOG_SINK_NONE           0
#define LOG_SINK_SERIAL         0x1
#define LOG_SINK_SD             0x2
/**
 * NOTE: Logging into SPIFFS is not a good idea, prefer to persist them into SD.
 * SPIFFS has reduced space and may fragment, suited for persisting few
 * controlled data-files.
 * Besides, the docs mention over a second write-time, occasionally:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spiffs.html#notes
 */
#define LOG_SINK_SPIFFS         0x4

#define STORAGE_NONE            0
/**
 * Store trip-traces into SD/SPIFFS;  SD & SPIFFS are mutual-exclusive.
 * NOTE: Storing trips delays boot due to enumerating old files.
 */
#define STORAGE_SPIFFS          1
#define STORAGE_SD              2

#define GNSS_NONE               0
#define GNSS_STANDALONE         1
#define GNSS_CELLULAR           2

#define PROTOCOL_UDP            1
#define PROTOCOL_HTTPS          2

#define PROTOCOL_METHOD_GET     0
#define PROTOCOL_METHOD_POST    1

#define LOG_EXT_SENSORS_NONE    0
#define LOG_EXT_SENSORS_DIGITAL 1
#define LOG_EXT_SENSORS_ANALOG  2

/**************************************
 * Logging (see also `platformio.ini`)
 **************************************/
/**
 * Works only when ESP_IDF logging-lib selected in `platformio.ini`.
 * (json-config default)
 */
#define RUNTIME_LOG_LEVELS \
    {"*", (esp_log_level_t)CORE_DEBUG_LEVEL},

    /** logs in `telelogger.cpp` */
    // {"SETUP", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"INIT", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"TELE", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"PROC", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs about BufMan & Buffers in `teleclient.h` */
    // {"BUF", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"NET", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in `FreematicsPlus.cpp` */
    // {"FreematicsPlus.cpp", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"LINK", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"GSM", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"GNSS", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"SPI", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in `libraries/FreematicsPlus/FreematicsNetwork.cpp` */
    // {"WIFI", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"SIM800", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"SIM5360", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"SIM7600", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"SIM7070", (esp_log_level_t)CORE_DEBUG_LEVEL),
    // {"NetHTTP", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in `libraries/FreematicsPlus/FreematicsOBD.cpp` */
    // {"OBD", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in `libraries/FreematicsPlus/FreematicsMEMS.cpp` */
    // {"MEMS", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in `libraries/Util/multilog.xxx` */
    // {"MULTILOG", (esp_log_level_t)CORE_DEBUG_LEVEL),
    /** logs in arduino-libs */
    // {"WiFiGeneric.cpp", (esp_log_level_t)CORE_DEBUG_LEVEL),


/**
 * Whether to enabled multiple log destinations (sinks).
 *
 * NOTE: enabling this without any `LOG_SINK_XX` will produce no logs at all!
 */
#define ENABLE_MULTILOG         0
/**
 * Which log destinations (sinks) to enable (relevant only if `ENABLE_MULTILOG`).
 * Either `LOG_SINK_NONE` or `LOG_SINK_XXX` constants OR-ed together.
 */
#define LOG_SINK                            LOG_SINK_SERIAL
 /** (json-config defaults) */
#define LOG_SINK_FPATH                      "/logs.txt"
#define LOG_SINK_DISK_USAGE_PURGE_RATIO     0.90f
#define LOG_SINK_SYNC_INTERVAL_MS           3141

/**
 * Secrets hidden:
 * - srv_XXX(URL fields): dump str-length
 * - ota_url: dump str-length
 * - wifi_ssids
 * - cell_apn: dump str-length
 * - sim_card_pin
 *
 * WARNING: URLs are printed on errors (eg no network connection),
 * so better disable logs completely for really mute firmware!
 *
 * See also `NodeInfo:hide_sensitive_node_infos()`
 */
#define HIDE_SECRETS_IN_LOGS    0

/**************************************
* OBD-II configurations
**************************************/
/**
 * When not 0, after boot the USB-uart is bidirectionally piped
 * directly to LINK-uart (connected to STM32F103CX co-processor's usart-2),
 * effectively allowing to send ELM327 AT-commands from the Serial <--> OBD.
 *
 * - The timeout resets on any Rx/Tx chars.
 * - Reboots after the timeout has expired AND any chars have Rx/Tx,
 *   otherwise, proceeds with regular setup.
 * - (json-config default)
 */
#define BOOT_OBD_PIPE_TIMEOUT_SEC    0

#ifndef ENABLE_OBD
#define ENABLE_OBD              1
#endif

/**
 * Maximum consecutive OBD access errors before entering standby.
 * (json-config default)
 */
#define MAX_OBD_ERRORS          3

/**************************************
 * Networking configurations
 **************************************
 * Don't modify per-device network settings & secrets here,
 * do it in `secrets.h` overrides instead:
 *      #define NET_DEVICE            NET_xxx   //(default below)
 *      #define WIFI_SSIDS            {"ssid1", "pswd1"}, ...
 *      #define CELL_APN ""
 *      #define SIM_CARD_PIN          ""
 *      #define SERVER_HOST           "hub.freematics.com"
 */
// change the following line to change network device
#define NET_DEVICE              NET_WIFI
/**
 * Known WiFi SSIDs is an initializer of `map<string, string>` expression,
 * like:
 *     {"ssid1", "pswd1"}, ...`
 *
 * - If empty, connects to the 1st open WiFi.
 * - If only one given, force-connects even if hidden.
 * - (json-config defaul)
 */
#define WIFI_SSIDS
/**
 * Cellular access-point name for network;  leave empty for all.
 * (json-config default)
 */
#define CELL_APN                ""
/** Freematics Hub server where to send collected data
 * (json-config default)
 */
#define SERVER_HOST             "hub.freematics.com"
#define SERVER_PROTOCOL         PROTOCOL_UDP

/**
 * SIM card setting
 * (json-config default)
 */
#define SIM_CARD_PIN            ""

// HTTPS settings
#define SERVER_METHOD           PROTOCOL_METHOD_POST
/**
 * The path-part of the url of the traccar server to send data to.
 * (json-config default)
 */
#define SERVER_PATH             "/hub/api"

/** (json-config default) */
#if !SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT             8081
#elif SERVER_PROTOCOL == PROTOCOL_HTTPS
#define SERVER_PORT             443
#endif
#endif

// WiFi Mesh settings
#define WIFI_MESH_ID            "123456"
#define WIFI_MESH_CHANNEL       13

// WiFi AP settings
#define WIFI_AP_SSID            "TELELOGGER"
#define WIFI_AP_PASSWORD        "PASSWORD"

/**
 * How many times to attempt opening net-connection before reporting error.
 * (json-config default)
 */
#define NET_CONNECT_RETRIES         5
/**
 * How much time to sleep before reattempting to net-connect.
 * (json-config default)
 */
#define UDP_CONNECT_RETRY_DELAY_MS  3000
/**
 * Maximum consecutive communication errors before resetting network.
 * (json-config default)
 */
#define MAX_CONN_ERRORS_RECONNECT   3
/**
 * Timeout for receiving an event response.
 * (json-config default)
 */
#define DATA_RECEIVING_TIMEOUT_MS   5000
/**
 * Expected maximum server sync signal interval.
*  Set 0 to disable
 * (json-config default)
 */
#define SERVER_SYNC_INTERVAL_SEC    120
/**
 * Intervals for when to send data based on stationary status.
 *
 * The values below are initializers for a `std::vector<stationary_interval_t>`
 * pairs like this:
 *
 *     {<stationary-duration-sec>, <transmission-interval-ms>}
 *
 * Τhe 1st interval(right-value) applies either when the vehicle is moving, or
 * when stationary for less than the 1st duration(left-value);
 * the rest pairs are expected to gradually increase the intervals (left-values)
 * as the stationary durations (right-values) increase,
 * until the last stationary duration (left-value),
 * which defines when the device should fall to standby.
 *
 * (json-config default)
 */
#define STATIONARY_TRANSMISSION_INTERVALS \
        {20, 1000}, \
        {40, 2000}, \
        {60, 5000},

/**
 * How often to ping the server?
 * (json-config default)
 */
#define PING_BACK_INTERVAL_SEC      900

/**
 * How often to send PIDs form the on-board fuel-consumption monitoring device.
 * (json-config default)
 */
#define OBFCM_INTERVAL_MS           30000

/**
 * How often to dump buffer & network statistics (both of them).
 * (json-config default)
 */
#define STATS_INTERVAL_SEC          12

/**************************************
* Data storage configurations
**************************************/
#ifndef STORAGE
// change the following line to change storage type
#define STORAGE                 STORAGE_NONE
#endif

/**************************************
* MEMS sensors
**************************************/
#define ENABLE_ORIENTATION      0
#ifndef ENABLE_MEMS
#define ENABLE_MEMS             1
#endif

/**************************************
* GPS
**************************************/
#ifndef GNSS
// change the following line to change GNSS setting
#define GNSS                    GNSS_STANDALONE
#endif
#define GPS_SERIAL_BAUDRATE     115200L
#define GPS_MOTION_TIMEOUT      180 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
/**
 * Whether to reset the device after waking up from "sleep".
 * (json-config default)
 */
#define REBOOT_ON_WAKEUP        0
 /* moving vehicle motion threshold in G */
#define MOTION_THRESHOLD        0.4f
// engine jumpstart voltage for waking up (when ENABLE_MEMS)
#define THR_VOLTAGE             13.6 /* V */
// engine jumpstart voltage gradient
#define THR_GRAD                1 /* V */

/**************************************
* Additional features
**************************************/
/**
 * Enable filesystem access commands?
 * The respective filesystem is implicitly enabled when
 * any of `STORAGE_XXX` or `MULTILINE+LOG_SINK_XXX` are used.
 */
#define ENABLE_SD               0
#define ENABLE_SPIFFS           0
#define FORMAT_SD_IF_FAILED     true
#define FORMAT_SPIFFS_IF_FAILED true

/** How long commands read from the serial can be? */
#define CMD_SERIAL_MAX_LEN      128
/** How many lines `HEAD` command prints? */
#define CMD_HEAD_NLINES         64
/** How many bytes the `TAIL` command to backtrack from the end-of-file? */
#define CMD_TAIL_NBYTES         -4096

/**
 * Over-the-air firmware-upgrade from HTTPS enabled?
 * Performed with "OTA[ url]" command,
 * where any `path` is appended at the end of the `OTA_UPDATE_URL`
 *
 * If enabled, MUST also define `OTA_UPDATE_URL` and `OTA_UPDATE_CERT_PEM`.
 */
#define ENABLE_OTA_UPDATE       0
/**
 * The HTTPS site to download the firmware from.
 * (json-config default)
 */
#define OTA_UPDATE_URL          ""
/**
 * The certificate-chain in pem format is needed here,
 * taken from, eg `/etc/letsencrypt/live/<server.url>/chain.pem`.
 * (json-config default)
 */
#define OTA_UPDATE_CERT_PEM     ""

/**
 * When enabled, significant states produce buzzing patterns,
 * as explained in `./README.md` file.
 */
#define ENABLE_BUZTICKS         1

// enable(1)/disable(0) OLED_SH1106 screen (if connected to the board).
#define ENABLE_OLED             0

/** (json-config defaults) */
#define PIN_SENSOR1             34
#define PIN_SENSOR2             26

#define LOG_EXT_SENSORS         LOG_EXT_SENSORS_NONE

/** (json-config default) */
#define COOLING_DOWN_TEMP       80 /* celsius degrees */
/** (json-config default) */
#define COOLING_DOWN_SLEEP_SEC  5 /* celsius degrees */

////////////////////////////
// NON-USER CONFIGS BELOW //
////////////////////////////

/** Include secrets & per device/user overrides */
#if __has_include("secrets.h")
#   include "secrets.h"
#endif

#define _NEED_SD        (ENABLE_SD || (STORAGE == STORAGE_SD) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG && (LOG_SINK & LOG_SINK_SD)))
#define _NEED_SPIFFS    (ENABLE_SPIFFS || (STORAGE == STORAGE_SPIFFS) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG && (LOG_SINK & LOG_SINK_SPIFFS)))

// Masked secrets to log instead.
//
extern const char apn2log[];
extern const char host2log[];
extern const int port2log;
extern const char ota_url2log[];

#ifndef BOARD_HAS_PSRAM
#define BOARD_HAS_PSRAM 0
#endif
#ifndef BOARD_HAS_PSRAM_HIGH
#define BOARD_HAS_PSRAM_HIGH 0
#endif

#define _CHECK_BUZTICKS          (node_info.macroflags & (1 << 4))

#endif // CONFIG_H_INCLUDED
