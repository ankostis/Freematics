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

 * #undef ENABLE_WIFI
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

#ifdef CONFIG_ENABLE_OBD
#define ENABLE_OBD CONFIG_ENABLE_OBD
#endif
#ifdef CONFIG_ENABLE_MEMS
#define ENABLE_MEMS CONFIG_ENABLE_MEMS
#endif
#ifdef CONFIG_GNSS
#define GNSS CONFIG_GNSS
#endif
#ifdef CONFIG_STORAGE
#define STORAGE CONFIG_STORAGE
#endif
#ifdef CONFIG_BOARD_HAS_PSRAM
#define BOARD_HAS_PSRAM 1
#endif
#ifdef CONFIG_ENABLE_WIFI
#define ENABLE_WIFI CONFIG_ENABLE_WIFI
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#endif
#ifdef CONFIG_ENABLE_BLE
#define ENABLE_BLE CONFIG_ENABLE_BLE
#endif
#ifdef CONFIG_ENABLE_HTTPD
#define ENABLE_HTTPD CONFIG_ENABLE_HTTPD
#endif
#ifdef CONFIG_SERVER_HOST
#define SERVER_HOST CONFIG_SERVER_HOST
#define SERVER_PORT CONFIG_SERVER_PORT
#define SERVER_PROTOCOL CONFIG_SERVER_PROTOCOL
#endif
#ifdef CONFIG_CELL_APN
#define CELL_APN CONFIG_CELL_APN
#endif

/**************************************
* Circular Buffer Configuration
**************************************/
#if BOARD_HAS_PSRAM
/**
 * Max number of buffers
 * If limit reached, the oldest slot is purged and re-populated,
 * hence, gaps & out-of-order rows appear in the trace.
 */
#define BUFFER_SLOTS            1024
/**
 * Bytes-per-slot
 * PID-samples not fitting in the current slot, they are dropped!
 */
#define BUFFER_LENGTH           384
#define SERIALIZE_BUFFER_SIZE   4096  /* bytes */
#else  //  No BOARD_HAS_PSRAM
#define BUFFER_SLOTS            256   /* see above */
#define BUFFER_LENGTH           180   /* see above */
#define SERIALIZE_BUFFER_SIZE   1024
#endif  // BOARD_HAS_PSRAM

/**************************************
* Configuration Definitions
**************************************/
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
#define PROTOCOL_HTTPS_GET      2
#define PROTOCOL_HTTPS_POST     3

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
 * (json-config default for `node_info.log_levels`)
 */
#define RUNTIME_LOG_LEVELS \
    {"*", (esp_log_level_t)CORE_DEBUG_LEVEL},

    /** TODO: check with list of log-TAGs in `telelogger.cpp` */
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
 * (json-config default for `node_info.log_levels`)
 */
#define LOG_SINK                            LOG_SINK_SERIAL
 /** (json-config default for `node_info.log_sink_fpath`) */
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
 * When not 0, after boot the USB-uart is bidirectionally piped directly to:
 * - LINK-uart (connected to STM32F103CX co-processor's usart-2), or to
 * - XBEE-uart (connected to SIMCOM modem),
 * effectively allowing the user terminal to send ELM327 or SIMCOM AT-commands
 * to speak directly to those modules.
 *
 * - The timeout resets on any Rx/Tx chars.
 * - Reboots after the timeout has expired AND any chars have Rx/Tx,
 *   otherwise, proceeds with regular setup.
 * - Old names: `CONFIG_MODE_TIMEOUT`, `BOOT_OBD_PIPE_TIMEOUT_SEC`
 * - (json-config default for `node_info.at_pipe_sec`)
 */
#define BOOT_AT_PIPE_TIMEOUT_SEC  0

#ifndef ENABLE_OBD
#define ENABLE_OBD                  1
#endif
/**
 * When 1, it generates & transmits speed=314kmh and a dummy VIN,
 * so as to keep communication to the server up and never sleep.
 * Use it to debug the network even when not hooked on a vehicle.
 *
 * ATTENTION: clean-before-build (for ./libraries/) needed when modified.
 */
#define ENABLE_OBD_EMULATION        0

/**
 * Maximum consecutive OBD access errors before entering standby.
 * (json-config default for `node_info.obd_max_errors`)
 */
#define MAX_OBD_ERRORS              3

/**
 * A list-of-list-of-alternate AT-cmds used to widen OBD masks/filter
 * when `OBD.init()` fails with ELM327-like coprocessor's defaults
 * ie. after sending `ATZ, ATEO, ATHO` and querying `010D` (`PID_SPEED`).
 *
 * Some usefull AT commands (more in `ELM327-AT_cmds.md` file):
 *
 * - ATSP 0 - Automaticaly detected by coprocessor
 * - ATSP 3 - ISO 9141-2
 * - ATSP 4 - ISO 14230-4 (KWP 5BAUD)
 * - ATSP 5 - ISO 14230-4 (KWP FAST)
 * - ATSP 6 - ISO 15765-4 (CAN 11/500)
 * - ATSP 7 - ISO 15765-4 (CAN 29/500)
 * - ATSP 8 - ISO 15765-4 (CAN 11/250)
 * - ATSP 9 - ISO 15765-4 (CAN 29/250)
 * - ATSP B - SAE J1939 CAN (29 bit ID, 250* kbaud)
 *
 * NOTE: some commands below (like `ATSH`) like `ATSP` to have run before
 * (maybe to decide frame's length?)
 *
 * - ATCM hhh/hhhhhhhh: Set the ID Mask (11bit/29bit CAN)
 * - ATCF hhh/hhhhhhhh: Set the ID Filter (11bit/29bit CAN)
 * - ATSH xyz/xxyyzz:   Set Header(11bit/29bit CAN)
 * - ATCP hh:           Set CAN Priority to hh (5msb of 29 bit)
 *
 * ## Regular PIDs & OBFCM IDs
 *
 * - 11bit:
 *   - BROADCAST:
 *     - 0x7DF (0b111_1110_1000)
 *   - typical (regular PIDs & OBFCM):
 *     - 0x7E8 (0b111_1110_1000)
 *     - 0x7E9 (0b111_1110_1001)
 *   - Toyota (OBFCM):
 *     - 0x7EA (0b111_1110_1010)
 * - 29bit:
 *   - BROADCAST:
 *     - 0x18DB33F1 (0b1_1000_1101_1011_0011_0011_1111_0001)
 *   - typical (regular PIDs & OBFCM):
 *     - 0x18DAF110 (0b1_1000_1101_1010_1111_0001_0001_0000)
 *   - Mercedes (regular PIDs):
 *     - 0x18DAF158 (0b1_1000_1101_1010_1111_0001_0101_1000)
 *     - 0x18DAF15A (0b1_1000_1101_1010_1111_0001_0101_1010)
 *     - 0x18DAF15D (0b1_1000_1101_1010_1111_0001_0101_1101)
 *   - Mercedes (OBFCM):
 *     - 0x18DAF159 (0b1_1000_1101_1010_1111_0001_0101_1001)
 *
 * ## Default Filters & Masks (baked into the ELM327 coproc)
 *
 * - 11 bit:
 *   - Accepts data from 7E8 & 7E9 but not 7EA.
 *   - filter: 0x7E8 (0b111_1110_1000)
 *   - mask:   0x7FE (0b111_1111_1110)
 * - 29bit:
 *   - accepts from 0x18DAF110 but not 0x18DAF158/9
 *   - filter: ??
 *   - mask:   ??
 *
 * (json-config default for `node_info.obd_alt_init_cmds`)
 */
////                            [MERCEDES FILTER/MASK]                     [WIDER FILTER/MASK]
#define OBD_ALT_INIT_CMDS       {"ATSP7", "ATCF18DAF101", "ATCM1FFFFF00"}, {"ATSP6", "ATCF7E8", "ATCM7F8"}
////                            [TOYOTA OBFCM F/M: 0b111_1110_1010/0b111_1111_1111]
#define OBFCM_CMD_LIST_START    {"ATSP6\r", "ATCF7EA\r", "ATCM7FF\r"}
////                            Note: mask become stricter afterwards!
#define OBFCM_CMD_LIST_END      {"ATSP6\r", "ATCF7E8\r", "ATCM7FF\r"}


/**************************************
 * Networking configurations
 **************************************
 * Don't modify per-device network settings & secrets here,
 * do it in `secrets.h` overrides instead:
 *      #define WIFI_SSIDS            {"ssid1", "pswd1"}, ...
 *      #define CELL_APN ""
 *      #define SIM_CARD_PIN          ""
 *      #define SERVER_HOST           "hub.freematics.com"
 */
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1
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
#endif
/**
 * Cellular access-point name for network;  leave empty for all.
 * (json-config default for `node_info.cell_apn`)
 */
#ifndef CELL_APN
#define CELL_APN                ""
#endif
/** Freematics Hub server where to send collected data
 * (json-config default for `node_info.srv_host`)
 */
#ifndef SERVER_HOST
#define SERVER_HOST             "hub.freematics.com"
#endif
#ifndef SERVER_PROTOCOL
#define SERVER_PROTOCOL         PROTOCOL_UDP
#endif

/**
 * SIM card setting
 * (json-config default for `node_info.sim_card_pin`)
 */
#define SIM_CARD_PIN            ""
// TODO: `APN_USERNAME` & `APN_PASSWORD` --> nodinfo
#define APN_USERNAME            NULL
#define APN_PASSWORD            NULL

/**
 * The path-part of the url of the traccar server to send data to.
 * (json-config default for `node_info.srv_path`)
 */
#define SERVER_PATH             "/hub/api"

/** (json-config default for `node_info.srv_port`) */
#if !SERVER_PORT
#undef SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT             8081
#else
#define SERVER_PORT             443
#endif
#endif  // SERVER_PROTOCOL ? PROTOCOL_UDP

// WiFi Mesh settings
#define WIFI_MESH_ID            "123456"
#define WIFI_MESH_CHANNEL       13

// WiFi AP settings
#define WIFI_AP_SSID            "TELELOGGER"
#define WIFI_AP_PASSWORD        "PASSWORD"

/**
 * How many times to attempt contacting the server before reporting error.
 * (json-config default for `node_info.net_retries`)
 */
#define NET_CONNECT_RETRIES         5
/**
 * How much time to sleep before reattempting to net-connect.
 * (json-config default for `node_info.net_udp_reconnect_delay_ms`)
 */
#define UDP_CONNECT_RETRY_DELAY_MS  3000
/**
 * Maximum consecutive communication errors before resetting network.
 * (json-config default for `node_info.reconnect_max_nerrors`)
 */
#define MAX_CONN_ERRORS_RECONNECT   5
/**
 * Timeout for receiving an event response.
 * (json-config default for `node_info.net_recv_timeout_ms`)
 */
#define DATA_RECEIVING_TIMEOUT_MS   5000
/**
 * Expected maximum server sync signal interval.
*  Set 0 to disable
 * (json-config default for `node_info.srv_sync_timeout_ms`)
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
 * (json-config default for `node_info.transmission_intervals`)
 */
#define STATIONARY_TRANSMISSION_INTERVALS \
        {10, 1000}, \
        {60, 2000}, \
        {180, 5000},

/**
 * How often to ping the server?
 * (json-config default for `node_info.ping_back_interval_sec`)
 */
#define PING_BACK_INTERVAL_SEC      900
/**
 * How often to check RSSI?
 * (TODO: `SIGNAL_CHECK_INTERVAL`  --> `nodeinfo.rssi_interval_sec`
 */
#define SIGNAL_CHECK_INTERVAL       10


/**
 * How often to send PIDs form the on-board fuel-consumption monitoring device.
 * (json-config default for `node_info.obfcm_interval`)
 */
#define OBFCM_INTERVAL_MS           30000

/**
 * How often to dump buffer & network statistics (both of them).
 * (json-config default for `node_info.net/buf_stats_interval_sec`)
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
#define GPS_SERIAL_BAUDRATE     115200L  // TODO: drop unused `GPS_SERIAL_BAUDRATE`.
#define GPS_MOTION_TIMEOUT      180 /* seconds */

// These are the partial args in parenthesis fed into 7070G's command:
//      AT+CGNSMOD: <gps>,[<glonas>,<beidu>,<galileo>,<qzss>]
#define GNSS_GLONAS             "1,0,0,0"
#define GNSS_BEIDU              "0,1,0,0"
#define GNSS_GALILEO            "0,0,1,0"
#define GNSS_QZSS               "0,0,0,1"
/**
 * What extra GNSS system to use to enhance GPS's precision.
 *
 * ATTENTION: clean-before-build (for ./libraries/) needed when modified.
 * (TODO: `GNSS_PRECISION_SYSTEM` -> nodeinfo.json-config default)
  */
#define GNSS_7070_2ND_SYSTEM    GNSS_GLONAS

/**
 * keeping GNSS power on during standby.
 * (TODO: `GNSS_ALWAYS_ON` -> nodeinfo.json-config default)
*/
#define GNSS_ALWAYS_ON 0
/**
 * GNSS reset timeout while no signal.
 * TODO: `GNSS_RESET_TIMEOUT` --> `node_info.gnss_timeout_ms`
 */

#define GNSS_RESET_TIMEOUT      300 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
/**
 * Whether to reset the device after waking up from "sleep".
 * (json-config default for `node_info.reboot_on_wakeup`)
 */
#define REBOOT_ON_WAKEUP        1
/**
 * Moving vehicle motion threshold in G.
 * (json-config default for `node_info.wakeup_motion_thr`)
 */
#define MOTION_THRESHOLD        0.4f
/**
 * Engine jumpstart upper voltage for waking up (when ENABLE_MEMS).
 * (TODO: `wakeup_jumpstart_thr` -> `wakeup_volt_thr`)
 * (json-config default for `node_info.wakeup_jumpstart_thr`)
 */
#define THR_VOLTAGE             13.6 /* V */
/**
 * Engine jumpstart voltage gradient threshold.
 * (TODO: `THR_GRAD` -> nodeinfo.wakeup_volt_grad_thr`)
 */
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
 * (json-config default for `node_info.ota_url`)
 */
#define OTA_UPDATE_URL          ""
/**
 * The certificate-chain in pem format is needed here,
 * taken from, eg `/etc/letsencrypt/live/<server.url>/chain.pem`.
 * (json-config default for `node_info.ota_update_cert_pem`)
 */
#define OTA_UPDATE_CERT_PEM     ""

/**
 * When enabled, significant states produce buzzing patterns,
 * as explained in `./README.md` file.
 */
#define ENABLE_BUZTICKS         1

// enable(1)/disable(0) OLED_SH1106 screen (if connected to the board).
#define ENABLE_OLED             0

/** (json-config defaults for `node_info.pin_sensor1/2`) */
#define PIN_SENSOR1             34
#define PIN_SENSOR2             26

#define LOG_EXT_SENSORS         LOG_EXT_SENSORS_NONE

/** (json-config default for `node_info.cool_temp`) */
#define COOLING_DOWN_TEMP       80 /* celsius degrees */
/** (json-config default for `node_info.cool_delay_sec`) */
#define COOLING_DOWN_SLEEP_SEC  5

////////////////////////////
// NON-USER CONFIGS BELOW //
////////////////////////////

/** Include secrets & per device/user overrides */
#if __has_include("secrets.h")
#   include "secrets.h"
#endif

#define _NEED_SD        (ENABLE_SD || (STORAGE == STORAGE_SD) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG_DEFINED && (LOG_SINK & LOG_SINK_SD)))
#define _NEED_SPIFFS    (ENABLE_SPIFFS || (STORAGE == STORAGE_SPIFFS) || \
        (ENABLE_MULTILOG && USE_ESP_IDF_LOG_DEFINED && (LOG_SINK & LOG_SINK_SPIFFS)))

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

// enable(1)/disable(0) http server
#ifndef ENABLE_HTTPD
#define ENABLE_HTTPD 0
#endif

// enable(1)/disable(0) BLE SPP server (for Freematics Controller App).
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif

#define _CHECK_BUZTICKS          (node_info.macroflags & (1 << 4))


#endif // CONFIG_H_INCLUDED
