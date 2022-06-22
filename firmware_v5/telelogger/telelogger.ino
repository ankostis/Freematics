/******************************************************************************
* Arduino sketch of a vehicle data data logger and telemeter for Freematics Hub
* Works with Freematics ONE+ Model A and Model B
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

#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <esp_log.h>
#include <json.hpp>
#include <FreematicsPlus.h>
#include <Buzzer.h>
#include "config.h"
#include "telelogger.h"
#include "NodeInfo.h"
#include "telemesh.h"
#include "teleclient.h"
#if _NEED_SD
#include "SD.h"
#endif  // _NEED_SD
#if _NEED_SPIFFS
#include "SPIFFS.h"
#endif  // _NEED_SPIFFS
#if _NEED_SD || _NEED_SPIFFS
#include "fsutil.h"
#endif  // _NEED_SD || _NEED_SPIFFS
#if ENABLE_MULTILOG && USE_ESP_IDF_LOG
#include "multilog.h"
#endif  // ENABLE_MULTILOG && USE_ESP_IDF_LOG
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// ESP_IDF logging tags of this file
constexpr const char TAG_BOOT[] = "BOOT";
constexpr const char TAG_INIT[] = "INIT";
constexpr const char TAG_TELE[] = "TELE";
constexpr const char TAG_PROC[] = "PROC";

/**
 * NOTE: changes here, must convey to platformIO's monitor-filter.
 */
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_NET_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_STANDBY 0x100
#define STATE_GET_OBFCM 0x200
#define STATE_SEND_ITID17 0x800
#define STATE_SEND_ITID1A 0x1000
#define STATE_SEND_ITID1B 0x2000
#define STATE_SEND_ITID1C 0x4000

DS_CAN_MSG obfcmData[]=
{
  {1,  OBFCM_TOTAL_DISTANCE_TRAVELED_RECENT, OBFCM_TOTAL_DISTANCE_TRAVELED_RECENT_ID,                4, 1, 9, 0,   32, 0.1,  0},
  {2,  OBFCM_TOTAL_DISTANCE_TRAVELED_LIFETIME, OBFCM_TOTAL_DISTANCE_TRAVELED_LIFETIME_ID,              4, 2, 9, 32,  32, 0.1,  0},
  {3,  OBFCM_TOTAL_FUEL_TRAVELED_RECENT, OBFCM_TOTAL_FUEL_TRAVELED_RECENT_ID,                    4, 3, 9, 64,  32, 0.01, 0},
  {4,  OBFCM_TOTAL_FUEL_TRAVELED_LIFETIME, OBFCM_TOTAL_FUEL_TRAVELED_LIFETIME_ID,                  4, 4, 9, 96,  32, 0.01, 0},
  {5,  OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_RECENT, OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_RECENT_ID,           6, 1, 9, 0,   32, 0.1,  0},
  {6,  OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_LIFETIME, OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_LIFETIME_ID,         6, 2, 9, 32,  32, 0.1,  0},
  {7,  OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_RECENT, OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_RECENT_ID,            6, 3, 9, 64,  32, 0.1,  0},
  {8,  OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_LIFETIME, OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_LIFETIME_ID,          6, 4, 9, 96,  32, 0.1,  0},
  {9,  OBFCM_PEV_DIST_CHARGE_INCREASING_RECENT, OBFCM_PEV_DIST_CHARGE_INCREASING_RECENT_ID,             6, 5, 9, 128, 32, 0.1,  0},
  {10, OBFCM_PEV_DIST_CHARGE_INCREASING_LIFETIME, OBFCM_PEV_DIST_CHARGE_INCREASING_LIFETIME_ID,           6, 6, 9, 160, 32, 0.1,  0},
  {11, OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_RECENT, OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_RECENT_ID,          4, 1, 9, 0,   32, 0.01, 0},
  {12, OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_LIFETIME, OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_LIFETIME_ID,        4, 2, 9, 32,  32, 0.01, 0},
  {13, OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_RECENT, OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_RECENT_ID,          4, 3, 9, 64,  32, 0.01, 0},
  {14, OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_LIFETIME, OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_LIFETIME_ID,        4, 4, 9, 96,  32, 0.01, 0},
  {15, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_RECENT, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_RECENT_ID,    6, 1, 9, 0,   32, 0.1,  0},
  {16, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_LIFETIME, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_LIFETIME_ID,  6, 2, 9, 32,  32, 0.1,  0},
  {17, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_RECENT, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_RECENT_ID,     6, 3, 9, 64,  32, 0.1,  0},
  {18, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_LIFETIME, OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_LIFETIME_ID,   6, 4, 9, 96,  32, 0.1,  0},
  {19, OBFCM_PEV_GRID_ENERGY_IN_BATTERY_RECENT, OBFCM_PEV_GRID_ENERGY_IN_BATTERY_RECENT_ID,             6, 5, 9, 128, 32, 0.1,  0},
  {20, OBFCM_PEV_GRID_ENERGY_IN_BATTERY_LIFETIME, OBFCM_PEV_GRID_ENERGY_IN_BATTERY_LIFETIME_ID,           6, 6, 9, 160, 32, 0.1,  0},
  {0}
};

DS_CAN_MSG obdDataMulti[]=
{
  {21, PID_RPM, PID_RPM_ID,                        1, 1, 1, 0,   16, 0.25, 0},
  {22, PID_ENGINE_LOAD, PID_ENGINE_LOAD_ID,                        1, 1, 1, 0,   8, 100/255, 0},
  {23, PID_ENGINE_FUEL_RATE, PID_ENGINE_FUEL_RATE_ID,                        1, 1, 1, 0,   16, 0.05, 0},
  {24, PID_ACC_PEDAL_POS_D, PID_ACC_PEDAL_POS_D_ID,                        1, 1, 1, 0,   8, 100/255, 0},
  {25, PID_HYBRID_BATTERY_PERCENTAGE, PID_HYBRID_BATTERY_PERCENTAGE_ID,                        1, 1, 1, 0,   8, 100/255, 0},
  {26, PID_COOLANT_TEMP, PID_COOLANT_TEMP_ID,                        1, 1, 1, 0,   8, 1, -40},
  {27, PID_AMBIENT_TEMP, PID_AMBIENT_TEMP_ID,                        1, 1, 1, 0,   8, 1, -40},
  {28, MULTIPID_ENGINE_FUEL_RATE_GS, MULTIPID_ENGINE_FUEL_RATE_GS_ID,                        2, 1, 1, 0,   16, 0.02, 0},
  {29, MULTIPID_VEHICLE_FUEL_RATE_GS, MULTIPID_VEHICLE_FUEL_RATE_GS_ID,                        2, 2, 1, 16,   16, 0.02, 0},
  {0}
};

node_info_t node_info{
  .enable_flags = (
    ((ENABLE_OBD && 1) << 0)
    | ((ENABLE_MEMS && 1) << 1)
    | ((ENABLE_ORIENTATION && 1) << 2)
    | ((ENABLE_OLED && 1) << 3)
    | ((ENABLE_BUZZING_INIT && 1) << 4)
    | ((ENABLE_OTA_UPDATE && 1) << 5)
    | ((_NEED_SD && 1) << 6)
    | ((_NEED_SPIFFS && 1) << 7)
    | ((ENABLE_MULTILOG && 1) << 8)
  | ((USE_ESP_IDF_LOG && 1) << 9)
    ),
  .serial_autoconf_timeout = CONFIG_MODE_TIMEOUT,
  .log_level_run = RUNTIME_ALL_TAGS_LOG_LEVEL,
  .log_level_build = CORE_DEBUG_LEVEL,
  .log_sink = LOG_SINK,
  .log_sink_fpath = LOG_SINK_FPATH,
  .log_sink_disk_usage_purge_prcnt = LOG_SINK_DISK_USAGE_PURGE_RATIO,
  .log_sink_sync_interval_ms = LOG_SINK_SYNC_INTERVAL_MS,
  .nslots = BUFFER_SLOTS,
  .slot_len = BUFFER_LENGTH,
  .serialize_len = SERIALIZE_BUFFER_SIZE,
  .storage = STORAGE,
  /** NOTE: changes here, must convey to platformIO's monitor-filter. */
  .gnss = GNSS,
  .ota_url = OTA_UPDATE_URL,
  .ota_update_cert_pem = OTA_UPDATE_CERT_PEM,
  .net_dev = NET_DEVICE,
  .wifi_ssd = (char*)WIFI_SSID,
  .wifi_pwd = (char*)WIFI_PASSWORD,
  .cell_apn = (char*)CELL_APN,
  .sim_card_pin = (char*)SIM_CARD_PIN,
  .srv_proto = SERVER_PROTOCOL,
  .srv_host = (char*)SERVER_HOST,
  .srv_path = (char*)SERVER_PATH,
  .srv_port = SERVER_PORT,
  .net_recv_timeout = DATA_RECEIVING_TIMEOUT,
  .srv_sync_timeout = SERVER_SYNC_INTERVAL,
  .net_retries = NET_CONNECT_RETRIES,
  .net_udp_reconnect_delay = UDP_CONNECT_RETRY_DELAY_MS,
  .stationary_timeout_vals = STATIONARY_TIME_TABLE,
  .data_interval_vals = DATA_INTERVAL_TABLE,
  .obfcm_interval = OBFCM_INTERVAL_MS,
  .obd_max_errors = MAX_OBD_ERRORS,
  .ping_back_interval = PING_BACK_INTERVAL,
  .wakeup_reset = RESET_AFTER_WAKEUP,
  .wakeup_motion_thr = MOTION_THRESHOLD,
  .wakeup_jumpstart_thr = THR_VOLTAGE,
  .cool_temp = COOLING_DOWN_TEMP,
  .cool_delay = COOLING_DOWN_SLEEP_SEC,
  .pin_sensor1 = PIN_SENSOR1,
  .pin_sensor2 = PIN_SENSOR2,
};

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
};

CBufferManager bufman;
Task subtask;

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
uint8_t accCount = 0;
#endif
float deviceTemp = 0;

// live data
int16_t rssi = 0;
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;
char obfcmTest[128] = {0};

char isoTime[26] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;
uint32_t lastObfcmTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;

bool serverSetup(IPAddress& ip);  // TODO: drop unused code
void serverProcess(int timeout);
String executeCommand(const char* cmd);
bool processCommand(char* data);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;
Buzzer buzzer(PIN_BUZZER);

#if ENABLE_OBD
class OBD : public COBD
{
protected:
  void idleTasks()  // TODO: drop obd `idle_tasks()` and run mems in a task.
  {
    // do some quick tasks while waiting for OBD response
#if ENABLE_MEMS
    processMEMS(0);
#endif
    delay(5);
  }
};

OBD obd;
#endif  // ENABLE_OBD
#if ENABLE_MEMS
MEMS_I2C* mems = nullptr;
#endif  // ENABLE_MEMS

#if (STORAGE == STORAGE_SPIFFS)
SPIFFSLogger logger;
#elif (STORAGE == STORAGE_SD)
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;

#   define OLED_CLEAR()      oled.clear()
#   define OLED_PRINT(...)      oled.print(__VA_ARGS__)
#   define OLED_PRINTF(...)     oled.printf(__VA_ARGS__)
#   define OLED_PRINTLN(...)    oled.println(__VA_ARGS__)
#   define OLED_SET_CURSOR(...) oled.setCursor(__VA_ARGS__)


#else // ENABLE_OLED
#   undef OLED_CLEAR
#   undef OLED_PRINT
#   undef OLED_PRINTF
#   undef OLED_PRINTLN
#   undef OLED_SET_CURSOR

#   define OLED_CLEAR()         do {} while(0)
#   define OLED_PRINT(...)      do {} while(0)
#   define OLED_PRINTF(...)     do {} while(0)
#   define OLED_PRINTLN(...)    do {} while(0)
#   define OLED_SET_CURSOR(...) do {} while(0)
#endif // ENABLE_OLED

State state;

void printTimeoutStats()
{
  ESP_LOGI(TAG, "Timeouts: OBD: %i, network: %i", timeoutsOBD, timeoutsNet);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
  int pins[] = {PIN_SENSOR1, PIN_SENSOR2};
  int pids[] = {PID_EXT_SENSOR1, PID_EXT_SENSOR2};
#if LOG_EXT_SENSORS == LOG_EXT_SENSORS_DIGITAL
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], digitalRead(pins[i]));
  }
#elif LOG_EXT_SENSORS == LOG_EXT_SENSORS_ANALOG
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], analogRead(pins[i]));
  }
#endif
}
#endif

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
#if ENABLE_OBD
void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, value);
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }

  obd.readPIDMulti(obdDataMulti);
  int idx_i=0;
  while (obdDataMulti[idx_i].idx) {
    buffer->add((uint16_t) obdDataMulti[idx_i].id, (int) obdDataMulti[idx_i].value);
    idx_i++;
  }

  int kph = obdData[0].value;
  if (kph >= 1) lastMotionTime = millis();
}
#endif

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  } else {
#if NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
    if (!teleClient.net.getLocation(&gd)) {
      return false;
    }
#endif
  }

  if (!gd || lastGPStime == gd->time || gd->date == 0 || (gd->lng == 0 && gd->lat == 0)) return false;

  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng > 0.001))) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = (float)((int)(gd->speed * 1.852f * 10)) / 10;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_DATE, gd->date);
    buffer->add(PID_GPS_TIME, gd->time);
    if (gd->lat && gd->lng && gd->alt) {
      buffer->add(PID_GPS_LATITUDE, gd->lat);
      buffer->add(PID_GPS_LONGITUDE, gd->lng);
      buffer->add(PID_GPS_ALTITUDE, gd->alt); /* m */
      buffer->add(PID_GPS_SPEED, kph);
      // buffer->add(PID_GPS_HEADING, gd->heading);
      buffer->add(PID_GPS_SAT_COUNT, gd->sat);
      buffer->add(PID_GPS_HDOP, gd->hdop);
    }
  }

  // generate ISO time string
  char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
      (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
      (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
  unsigned char tenth = (gd->time % 100) / 10;
  if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
  *p = 'Z';
  *(p + 1) = 0;

  ESP_LOGD(TAG_PROC,
      "<GPS> lat: %.6f, lng: %.6f, v: %.1f km/h, heading: %i, sats: %i"
      ", t: %s, err: %i",
      gd->lat, gd->lng, kph, gd->heading, gd->sat, isoTime, gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#define ENABLE_MOVEMENT 0
#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  // load and store accelerometer data
  float temp = 0;
  float acc[3];
  float gyr[3];
  float mag[3];
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
#endif

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  if (buffer) {
    if (accCount) {
#if ENABLE_ORIENTATION || ENABLE_MOVEMENT
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      // buffer->add(PID_ACC, value);
#endif
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, value);
#endif
      if (temp != deviceTemp) {
        deviceTemp = temp;
        buffer->add(PID_DEVICE_TEMP, (int)temp);
      }
#if ENABLE_MOVEMENT
      // calculate motion
      float motion = 0;
      for (byte i = 0; i < 3; i++) {
        motion += value[i] * value[i];
      }
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        lastMotionTime = millis();
        ESP_LOGD(TAG_PROC, "Motion: %.4f", motion);
      }
#endif
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    ESP_LOGD(TAG_PROC, "ACC BIAS: %.2f/%.2f/%.2f", accBias[0], accBias[1], accBias[2]);
  }
}
#endif

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    ESP_LOGI(TAG, "UTC: %s", buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
  buzzer.tone(2000);
  delay(100);
  buzzer.tone(4, 0.5);  // 2Hz ta-ticks till `initialize()` ends.

  // dump buffer data
  bufman.purge();

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_STANDALONE
  // start GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    if (sys.gpsBegin(GPS_SERIAL_BAUDRATE)) {
      state.set(STATE_GPS_READY);
      ESP_LOGI(TAG_INIT, "GNSS: OK, state: %X", state.m_state);
      OLED_PRINTLN("GNSS OK");
    } else {
      ESP_LOGE(TAG_INIT, "GNSS: NO, state: %X", state.m_state);
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      state.set(STATE_OBD_READY | STATE_GET_OBFCM);
      ESP_LOGI(TAG_INIT, "OBD: OK, state: %X", state.m_state);
      OLED_PRINTLN("OBD OK");
    } else {
      ESP_LOGE(TAG_INIT, "OBD:NO");
      //state.clear(STATE_WORKING);
      //return;
    }
  }
#endif

#if 0
  if (!state.check(STATE_OBD_READY) && state.check(STATE_GPS_READY)) {
    // wait for movement from GPS when OBD not connected
    Serial.println("Waiting...");
    if (!waitMotionGPS(GPS_MOTION_TIMEOUT * 1000)) {
      return false;
    }
  }
#endif

#if STORAGE
  if (!state.check(STATE_STORAGE_READY)) {
    // init storage
    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
  }
#endif

  // re-try OBD if connection not established
#if ENABLE_OBD
  char *vin = node_info.vin;

  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      memcpy(vin, buf, sizeof(vin) - 1);
      ESP_LOGI(TAG_INIT, "VIN: %s", vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      ESP_LOGI(TAG_INIT, "DTC: %i", dtcCount);
    }
    int i = 0;
    obd.GetOBFCM(obfcmData);
    while (obfcmData[i].idx){
      ESP_LOGV(TAG_INIT, "OBFCM %i: %i", i, obfcmData[i].value);
      i++;
    }

    OLED_PRINTF("VIN: %s\n", vin);
  }
#endif

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);  // This `delay()` forces #ifdef OLED.
  oled.clear();
  oled.printf("DEVICE ID: %s\n", node_info.device_id);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif

  teleClient.notify(EVENT_PING); // Nestor

  buzzer.tone(0);  // stop ticking
  ESP_LOGI(TAG_INIT, "completed");
}

/*******************************************************************************
  Executing a remote command
*******************************************************************************/
#if ENABLE_OTA_UPDATE
#include <esp_https_ota.h>

/**
 * :param ota_url:
 *    if null, read from `node_info`.
 */
void do_firmware_upgrade(const char *ota_url = nullptr) {
  esp_http_client_config_t http_cfg = {
      .url = ota_url? ota_url : node_info.ota_url,
      .cert_pem = node_info.ota_update_cert_pem,
  };
  const char* url2log = (ota_url ? ota_url : ota_url2log);
  ESP_LOGI(TAG_PROC, "OTA update from %s...", url2log);
  esp_err_t ret = esp_https_ota(&http_cfg);
  ESP_LOG_LEVEL(
      (ret == ESP_OK? ESP_LOG_INFO : ESP_LOG_ERROR),
      TAG_PROC,
      "OTA updated from %s OK.\n  Rebooting.",
      url2log);
  if (ret == ESP_OK) esp_restart();
}
#endif // ENABLE_OTA_UPDATE

String executeCommand(const char* cmd)
{
  String result;
  ESP_LOGI(TAG_PROC, "cmd: %s", cmd);
  if (!strncmp(cmd, "LED ", 4) && cmd[4]) {
    ledMode = (byte)atoi(cmd + 4);
    digitalWrite(PIN_LED, (ledMode == 2) ? HIGH : LOW);
    result = "OK";

  } else if (!strcmp(cmd, "REBOOT")) {
  #if STORAGE
    if (state.check(STATE_STORAGE_READY)) {
      logger.end();
      state.clear(STATE_STORAGE_READY);
    }
  #endif
    teleClient.shutdown();
    ESP.restart();
    // never reach here

#if ENABLE_OTA_UPDATE
  } else if (!strcmp(cmd, "OTA") || !strncmp(cmd, "OTA ", 4)) {
    const char *ota_url = (cmd[3] && cmd[4]) ? cmd + 4 : nullptr;
    do_firmware_upgrade(ota_url);  // Reboots on success.
    result = "ERROR";
#endif  // ENABLE_OTA_UPDATE

  } else if (!strcmp(cmd, "STANDBY")) {
    state.clear(STATE_WORKING);
    result = "OK";

  } else if (!strcmp(cmd, "WAKEUP")) {
    state.clear(STATE_STANDBY);
    result = "OK";

  } else if (!strncmp(cmd, "SET ", 4) && cmd[4]) {
    const char* subcmd = cmd + 4;

    if (!strncmp(subcmd, "SYNC ", 5) && subcmd[5]) {
      syncInterval = atoi(subcmd + 5);
      result = "OK";
    } else {
      result = "ERROR";
    }

  } else if (!strcmp(cmd, "STATS")) {
    char buf[64];
    sprintf(buf, "TX:%u OBD:%u NET:%u", teleClient.txCount, timeoutsOBD, timeoutsNet);
    result = buf;

#if ENABLE_OBD
  } else if (!strncmp(cmd, "OBD ", 4) && cmd[4]) {
    // send OBD command
    char buf[256];
    sprintf(buf, "%s\r", cmd + 4);
    if (obd.link && obd.link->sendCommand(buf, buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0) {
      result = buf;
    } else {
      result = "ERROR";
    }
#endif  // ENABLE_OBD

#if _NEED_SD || _NEED_SPIFFS
  /**************************
   * # Filesystem commands
   *
   * Try:
   *    CAT /sd/logs.txt
   *    TAIL /spiffs/logs.txt
   */

  } else if (!strcmp(cmd, "LS")) {
    std::stringstream out;

    File f = SD.open("/");
    out << "\nSD: ";
    listDir(f, out);

    f = SPIFFS.open("/");
    out << "\nSPIFFS: ";
    listDir(f, out);

    std::cout << out.str();
    result = "OK";

  } else if (!strncmp(cmd, "CAT ", 4) && cmd[4]) {
    std::ifstream ifs(cmd + 4);
    if (ifs.is_open()) {
      std::cout << ifs.rdbuf();
      result = "OK";
    } else {
      result = "ERROR";
    }

  } else if (!strncmp(cmd, "HEAD ", 5) && cmd[5]) {
    std::ifstream ifs(cmd + 5);
    if (ifs.is_open()) {
      std::string line;
      for (int i = 0; i < (CMD_HEAD_NLINES) && ifs; i++) {
        std::getline(ifs, line);
        if(ifs) std::cout << line << std::endl;
      }
      result = "OK";
    } else {
      result = "ERROR";
    }

  } else if (!strncmp(cmd, "TAIL ", 5) && cmd[5]) {
    std::ifstream ifs(cmd + 5);
    if (ifs.is_open()) {
      ifs.seekg(CMD_TAIL_NBYTES, ifs.end);
      std::cout << ifs.rdbuf();
      result = "OK";
    } else {
      result = "ERROR";
    }

  } else if (!strncmp(cmd, "RM ", 3) && cmd[3]) {
    result = std::remove(cmd + 3)? "ERROR" : "OK";
#endif // _NEED_SD || _NEED_SPIFFS

  } else {
    result = "INVALID CMD";
  }

  return result;
}

bool processCommand(char* data)
{
  char *p;
  if (!(p = strstr(data, "TK="))) return false;
  uint32_t token = atol(p + 3);
  if (!(p = strstr(data, "CMD="))) return false;
  char *cmd = p + 4;

  if (token > lastCmdToken) {
    // new command
    String result = executeCommand(cmd);
    // send command response
    char buf[256];
    snprintf(buf, sizeof(buf), "TK=%u,MSG=%s", token, result.c_str());
    for (byte attempts = 0; attempts < 3; attempts++) {
      if (teleClient.notify(EVENT_ACK, buf)) {
        ESP_LOGD(TAG_PROC, "ACK notified");
        break;
      } else {
        ESP_LOGW(TAG_PROC, "ACK ignored");
      }
    }
  } else {
    // previously executed command
    char buf[64];
    snprintf(buf, sizeof(buf), "TK=%u,DUP=1", token);
    for (byte attempts = 0; attempts < 3; attempts++) {
      if (teleClient.notify(EVENT_ACK, buf)) {
        ESP_LOGD(TAG_PROC, "ACK notified");
        break;
      } else {
        ESP_LOGW(TAG_PROC, "ACK ignored");
      }
    }
  }
  return true;
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  // unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {

    serverProcess(100);
    // calculate relative movement
    float motion = 0;
    float acc[3];
    if (!mems->read(acc)) {
      return false;
    }
    if (accCount == 10) {
      accCount = 0;
      accSum[0] = 0;
      accSum[1] = 0;
      accSum[2] = 0;
    }
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    accCount++;
    for (byte i = 0; i < 3; i++) {
      float m = (acc[i] - accBias[i]);
      motion += m * m;
    }
    // check movement
    if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
      //lastMotionTime = millis();
      return true;
    } else {
      return false;
    }
  }
#endif
  serverProcess(timeout);
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.get();
  if (!buffer) {
    buffer = bufman.getOldest();
    if (!buffer) return;
    while (buffer->state == BUFFER_STATE_LOCKED) delay(1);
    buffer->purge();
  }

  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {

    processOBD(buffer);

    if (state.check(STATE_GET_OBFCM)) {
      lastObfcmTime = millis();
      obd.GetOBFCM(obfcmData);
      state.clear(STATE_GET_OBFCM);
      state.set(STATE_SEND_ITID17 | STATE_SEND_ITID1A | STATE_SEND_ITID1B | STATE_SEND_ITID1C);
    }

    if (state.check(STATE_SEND_ITID17)) {
      buffer->add((uint16_t) 0x9172, (int) obfcmData[1].value);
      buffer->add((uint16_t) 0x9174, (int) obfcmData[3].value);
      state.clear(STATE_SEND_ITID17);
    } else if (state.check(STATE_SEND_ITID1A)) {
      buffer->add((uint16_t) 0x91A2, (int) obfcmData[5].value);
      buffer->add((uint16_t) 0x91A4, (int) obfcmData[7].value);
      buffer->add((uint16_t) 0x91A6, (int) obfcmData[9].value);
      state.clear(STATE_SEND_ITID1A);
    } else if (state.check(STATE_SEND_ITID1B)) {
      buffer->add((uint16_t) 0x91B2, (int) obfcmData[11].value);
      buffer->add((uint16_t) 0x91B4, (int) obfcmData[13].value);
      state.clear(STATE_SEND_ITID1B);
    } else if (state.check(STATE_SEND_ITID1C)) {
      // buffer->add((uint16_t) 0x91C2, (int) obfcmData[15].value);
      // buffer->add((uint16_t) 0x91C4, (int) obfcmData[17].value);
      buffer->add((uint16_t) 0x91C6, (int) obfcmData[19].value);
      state.clear(STATE_SEND_ITID1C);
    }

    if (millis() - lastObfcmTime > OBFCM_INTERVAL_MS) {
      ESP_LOGD(
        TAG_PROC, "Elapsed %.2fsec since last OBFCM.", (float)OBFCM_INTERVAL_MS / 1000);
      state.set(STATE_GET_OBFCM);
    }

    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        ESP_LOGE(TAG_PROC, "OBD OFF!");
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  }
#else
  buffer->add(PID_DEVICE_HALL, readChipHallSensor() / 200);
#endif

#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 12 * 370) / 4095;
  } else {
    batteryVoltage = obd.getVoltage() * 100;
  }
  if (batteryVoltage) {
    buffer->add(PID_BATTERY_VOLTAGE, batteryVoltage);
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  processGPS(buffer);

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
    buffer->add(PID_DEVICE_TEMP, deviceTemp);
  }

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;

  // display file buffer stats
  if (startTime - lastStatsTime >= 3000) {
    bufman.showCacheStats(state.m_state);
    lastStatsTime = startTime;
  }

#if STORAGE
  if (state.check(STATE_STORAGE_READY)) {
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      ESP_LOGI(TAG_PROC, "<FILE> %iKB", sizeKB);
    }
  }
#endif

#if ENABLE_OBD || ENABLE_MEMS
  // motion adaptive data interval control
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  const int dataIntervals[] = DATA_INTERVAL_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    // stationery timeout
    // trip ended if OBD is not available
    if (!state.check(STATE_OBD_READY)) state.clear(STATE_WORKING);
    ESP_LOGI(TAG_PROC,
        "Stationary for %i sec, state: %X",
        motionless, state.m_state);
  }
#endif
  long t = dataInterval - (millis() - startTime);
  if (t > 0 && t < dataInterval) delay(t);
}

bool initNetwork()
{
#if NET_DEVICE == NET_WIFI
  OLED_PRINT("Connecting WiFi...");
  for (byte attempts = 0; attempts < 3; attempts++) {
    ESP_LOGI(TAG_NET, "Joining WiFi: %s", WIFI_SSID);
    teleClient.net.begin(WIFI_SSID, WIFI_PASSWORD);
    if (teleClient.net.setup()) {
      state.set(STATE_NET_READY);
      String ip = teleClient.net.getIP();
      if (ip.length()) {
        state.set(STATE_NET_CONNECTED);
        ESP_LOGI(TAG_NET, "WiFi IP: %s", ip.c_str());
        OLED_PRINTLN(ip);
        break;
      }
    } else {
      ESP_LOGE(TAG_NET, "No WiFi");
      teleClient.net.listAPs();
    }
  }  // wifi attempts loop
#else // NET_DEVICE != NET_WIFI
  // power on network module
  if (teleClient.net.begin(&sys)) {
    state.set(STATE_NET_READY);
  } else {
    ESP_LOGE(TAG_NET, "CELL: NO");
    OLED_PRINTLN("No Cell Module");
    return false;
  }
#if NET_DEVICE >= SIM800
    OLED_PRINTF("%s OK\nIMEI: %s", teleClient.net.deviceName(), teleClient.net.IMEI);
  ESP_LOGI(TAG_NET, "CELL: %s", teleClient.net.deviceName());
  if (!teleClient.net.checkSIM(SIM_CARD_PIN)) {
    ESP_LOGE(TAG_NET, "NO SIM CARD");
    return false;
  }
  ESP_LOGI(TAG_NET, "IMEI: %s", teleClient.net.IMEI);
  if (state.check(STATE_NET_READY) && !state.check(STATE_NET_CONNECTED)) {
    if (teleClient.net.setup(CELL_APN)) {
      String op = teleClient.net.getOperatorName();
      if (op.length()) {
        ESP_LOGI(TAG_NET, "Operator: %s", op.c_str());
        OLED_PRINTLN(op);
      }

#if GNSS == GNSS_CELLULAR
      if (teleClient.net.setGPS(true)) {
        ESP_LOGI(TAG_NET, "CELL GNSS:OK");
      }
#endif

      String ip = teleClient.net.getIP();
      if (ip.length()) {
        ESP_LOGI(TAG_NET, "IP: %s", ip.c_str());
        OLED_PRINTF("IP: %s\n",ip.c_str());
      }
      rssi = teleClient.net.getSignal();
      if (rssi) {
        ESP_LOGI(TAG_NET, "CELL RSSI: %idBm", rssi);
        OLED_PRINTF("RSSI: %idBm\n", rssi);
      }
      state.set(STATE_NET_CONNECTED);
    } else {
      char *p = strstr(teleClient.net.getBuffer(), "+CPSI:");
      if (p) {
        char *q = strchr(p, '\r');
        if (q) *q = 0;
        ESP_LOGD(TAG_NET, "raw info: %s", p + 7);
        OLED_PRINTLN(p + 7);
      } else {
        ESP_LOGD(TAG_NET, "raw buf: %s", teleClient.net.getBuffer());
      }
    }
    timeoutsNet = 0;
  }
#else
  state.set(STATE_NET_CONNECTED);
#endif
#endif // NET_DEVICE == NET_WIFI
  return state.check(STATE_NET_CONNECTED);
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(SERIALIZE_BUFFER_SIZE);
  teleClient.reset();

  for (;;) {
    ESP_LOGD(TAG_TELE, "state: %X", state.m_state);
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_NET_READY)) {
        teleClient.shutdown();
      }
      state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      teleClient.reset();
      bufman.purge();

#if GNSS == GNSS_STANDALONE
      if (state.check(STATE_GPS_READY)) {
        sys.gpsEnd();
        state.clear(STATE_GPS_READY);
      }
      gd = 0;
#endif

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        // start ping
#if GNSS == GNSS_STANDALONE
        if (sys.gpsBegin(GPS_SERIAL_BAUDRATE)) {
          state.set(STATE_GPS_READY);
          for (uint32_t t = millis(); millis() - t < 120000; ) {
            if (sys.gpsGetData(&gd)) {
              break;
            }
          }
        }
#endif
        if (initNetwork()) {
          ESP_LOGV(TAG_TELE, "Ping...");
          bool success = teleClient.ping();
          ESP_LOGD(TAG_TELE, "Ping: %s", success ? "OK" : "NO");
        }
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      }
      continue;
    }

    if (!state.check(STATE_NET_CONNECTED)) {
      if (!initNetwork() || !teleClient.connect()) {
        buzzer.tone(1, 0.2);  // 0.5Hz ta-tick while waiting to reconnect.
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
        delay(10000);
        buzzer.tone(0);
        continue;
      }
    }

    connErrors = 0;
    teleClient.startTime = millis();

    for (;;) {
      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        if (!state.check(STATE_WORKING)) break;
        delay(50);
        continue;
      }

      buffer->state = BUFFER_STATE_LOCKED;
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(node_info.device_id);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      buffer->purge();
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.tailer();
#endif
      ESP_LOGD(TAG_TELE, "To tx: |%s|", store.buffer());

      // start transmission
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
      if (teleClient.transmit(store.buffer(), store.length())) {
        // successfully sent
        connErrors = 0;
        char timestr[16];
        teleClient.showNetStats(timestr, state.m_state);
        OLED_SET_CURSOR(0, 2);
        OLED_PRINTLN(timestr);
        OLED_PRINTF("%2i", teleClient.txCount);
        OLED_SET_CURSOR(80, 5);
        OLED_PRINTF("%3i", teleClient.txBytes >> 10);

      } else {
        connErrors++;
        timeoutsNet++;
        printTimeoutStats();
      }
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);

      store.purge();

      teleClient.inbound();
      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        ESP_LOGW(TAG_TELE, "Unstable connection");
        connErrors++;
        timeoutsNet++;
      }
      if (connErrors > MAX_CONN_ERRORS_RECONNECT) {
        ESP_LOGE(TAG_TELE, "Shutdown due to %i network errors!", connErrors);
        teleClient.net.close();
        if (!teleClient.connect()) {
          teleClient.shutdown();
          state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        // device too hot, cool down by pause transmission
        ESP_LOGE(TAG_TELE, "Overheat %.2f!  Purging cached samples.", deviceTemp);
        delay(COOLING_DOWN_SLEEP_SEC * 1000);
        bufman.purge();
      }

    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
#if STORAGE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif
  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
  state.set(STATE_STANDBY | STATE_GET_OBFCM);
  ESP_LOGI(TAG_PROC, "STANDBY(state: %X)", state.m_state);
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);  // This `delay()` forces #ifdef OLED.
  oled.clear();
#endif
  // this will put co-processor into sleep mode
#if ENABLE_OBD
  obd.enterLowPowerMode();
#endif  // ENABLE_OBD
#if ENABLE_MEMS
  uint32_t t;
  float v;
  uint32_t t_old = millis();
  float v_old = 999;
  float v_grad;

  calibrateMEMS();

  do {
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
    t = millis();
#if ENABLE_OBD
    v = obd.getVoltage();
#else  // ENABLE_OBD
    v = THR_VOLTAGE;
#endif  // ENABLE_OBD

    v_grad = (v - v_old)/(t - t_old)*1000;
    ESP_LOGI(
        TAG_PROC,
        "Wake-up trigger dt: %.2fs"
        ", Voltage old: %.2f, Voltage new: %.2f, Gradient: %.2f"
        ", state: %X"
        , (float)(t - t_old) / 1000
        , v_old
        , v
        , v_grad
        , state.m_state);
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);

    if (waitMotion(-1)) {
      break;
    }

    if (v*v_old < 0.1) continue;

    delay(880);
    t_old = t;
    v_old = v;
  } while (!((v > THR_VOLTAGE) && (v_grad > THR_GRAD)));
#elif ENABLE_OBD
  do {
    delay(1000);
  } while (obd.getVoltage() < THR_VOLTAGE);
#else
  delay(5000);
#endif
  ESP_LOGI(TAG_PROC, "Wakeup!");

#if RESET_AFTER_WAKEUP
#if ENABLE_MEMS
  mems->end();
#endif
  ESP.restart();
#endif
  state.clear(STATE_STANDBY);
  // this will wake up co-processor
  sys.resetLink();
  delay(200);
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
#if CONFIG_MODE_TIMEOUT
void configMode()
{
  uint32_t t = millis();

  do {
    if (Serial.available()) {
      // enter config mode
      ESP_LOGI(TAG, "#CONFIG MODE#");
      Serial1.begin(LINK_UART_BAUDRATE, SERIAL_8N1, PIN_LINK_UART_RX, PIN_LINK_UART_TX);
      do {
        if (Serial.available()) {
          Serial1.write(Serial.read());
          t = millis();
        }
        if (Serial1.available()) {
          Serial.write(Serial1.read());
          t = millis();
        }
      } while (millis() - t < CONFIG_MODE_TIMEOUT);
      ESP_LOGD(TAG, "#RESET#");
      delay(100);
      ESP.restart();
    }
  } while (millis() - t < CONFIG_MODE_TIMEOUT);
}
#endif

#if _NEED_SD
bool setup_SD()
{
    if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ, "/sd", 5/* files open */, FORMAT_SD_IF_FAILED)) {
        uint64_t total = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        float used_ratio = 100.0 * used / total;
        ESP_LOGI(TAG, "SD: %u MB total, %u MB used (%%%.2f)",
                 (uint)(total >> 20), (uint)(used >> 20), used_ratio);
        return true;
    } else {
        ESP_LOGE(TAG, "No SD card");
        return false;
    }
}
#endif // _NEED_SD

#if _NEED_SPIFFS
bool setup_SPIFFS()
{
    if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        uint64_t total = SPIFFS.totalBytes();
        uint64_t used = SPIFFS.usedBytes();
        float used_ratio = 100.0 * used / total;
        ESP_LOGI(TAG, "SPIFFS: %llu bytes total, %llu bytes used (%%%.2f)",
                total, used, used_ratio);
        return true;
    } else {
        ESP_LOGE(TAG, "No SPIFFS");
        return false;
    }
}
#endif // _NEED_SPIFFS

#if ENABLE_MULTILOG && USE_ESP_IDF_LOG
void setup_multilog() {
#if (LOG_SINK & LOG_SINK_SERIAL)
    static multilog::SerialSink serial_link;
    multilog::log_sinks[0] = &serial_link;
#endif  // (LOG_SINK & LOG_SINK_SERIAL)
#if (LOG_SINK & LOG_SINK_SD)
    static multilog::FileSink sd_sink{
            "SD",
            SD,
            node_info.log_sink_fpath,
            FILE_APPEND,
            node_info.log_sink_disk_usage_purge_prcnt,
            node_info.log_sink_sync_interval_ms,
    };
    multilog::log_sinks[1] = &sd_sink;
#endif
#if (LOG_SINK & LOG_SINK_SPIFFS)
    static multilog::FileSink spiffs_sink{
            "SPIFFS",
            SPIFFS,
            node_info.log_sink_fpath,
            FILE_APPEND,
            node_info.log_sink_disk_usage_purge_prcnt,
            node_info.log_sink_sync_interval_ms,
    };
    multilog::log_sinks[2] = &spiffs_sink;
#endif

    multilog::enable(true);
}  // setup_multilog()
#endif  // ENABLE_MULTILOG && USE_ESP_IDF_LOG

// Allow test-cases to call functions from here, and avoid clashing.
// see: https://docs.platformio.org/en/latest/advanced/unit-testing/structure/shared-code.html#unit-testing-shared-code
#ifndef PIO_UNIT_TESTING
void setup()
{
    buzzer.tone(1);  // 1hz ticks until `initialize()`
    delay(500);

#if ENABLE_OLED
    oled.begin();
    oled.setFontSize(FONT_SIZE_SMALL);
#endif
    // initialize USB serial
    Serial.begin(115200);

    // Relevant only when ESP_IDF log-lib selected (`USE_ESP_IDF_LOG=1`).
    //
    esp_log_level_set("*", (esp_log_level_t)node_info.log_level_run);
    // esp_log_level_set(TAG_BOOT, ESP_LOG_WARN);     // logs in this file
    // esp_log_level_set(TAG_INIT, ESP_LOG_WARN);     // logs in this file
    // esp_log_level_set(TAG_TELE, ESP_LOG_WARN);     // logs in this file
    // esp_log_level_set(TAG_PROC, ESP_LOG_WARN);     // logs in this file
    // esp_log_level_set(TAG_MULTILOG, ESP_LOG_WARN);
    // esp_log_level_set(TAG_LINK, ESP_LOG_WARN);     // OBD(& GNSS?) (soft?)UART
    // esp_log_level_set(TAG_GSM, ESP_LOG_WARN);      // GSM UART
    // esp_log_level_set(TAG_GNSS, ESP_LOG_WARN);     // GNSS (if not through LINK)
    // esp_log_level_set(TAG_SPI, ESP_LOG_WARN);      // MEMs(& LINK?) SPI
    // esp_log_level_set(TAG_BUF, ESP_LOG_WARN);      // BufMan & Buffers @ teleclient.h
    // esp_log_level_set(TAG_NET, ESP_LOG_WARN);      // TeleClientXXX & netInit()
    // esp_log_level_set(TAG_MOTION, ESP_LOG_WARN);   // MEMs check-motion
    // esp_log_level_set(TAG_WIFI, ESP_LOG_WARN);     // FreematcisNetwork
    // esp_log_level_set(TAG_SIM800, ESP_LOG_WARN);   // FreematcisNetwork
    // esp_log_level_set(TAG_SIM5360, ESP_LOG_WARN);  // FreematcisNetwork
    // esp_log_level_set(TAG_SIM7600, ESP_LOG_WARN);  // FreematcisNetwork
    // esp_log_level_set(TAG_SIM7070, ESP_LOG_WARN);  // FreematcisNetwork
    // esp_log_level_set(TAG_HTTP, ESP_LOG_WARN);     // FreematcisNetwork

#if _NEED_SD
    setup_SD();
#endif  // _NEED_SD
#if _NEED_SPIFFS
    setup_SPIFFS();
#endif  // _NEED_SPIFFS

#if ENABLE_MULTILOG
#if USE_ESP_IDF_LOG
    setup_multilog();
#else
    ESP_LOGW(TAG_INIT, "ENABLE_MULTILOG needs USE_ESP_IDF_LOG=1 to work!");
#endif  // USE_ESP_IDF_LOG
#endif  // ENABLE_MULTILOG

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

   // generate unique device ID
    mac_to_device_id(ESP.getEfuseMac(), node_info.device_id);  // TODO: encapsulate
    std::string hw_infos = generate_node_infos();

#if CONFIG_MODE_TIMEOUT
    configMode();
#endif

#if LOG_EXT_SENSORS
    pinMode(PIN_SENSOR1, INPUT);
    pinMode(PIN_SENSOR2, INPUT);
#endif
    nlohmann::json infos_j = node_info_to_json(node_info);
#if HIDE_SECRETS_IN_LOGS
    erase_sensitive_fields(infos_j);
#endif // HIDE_SECRETS_IN_LOGS
    ESP_LOGE(TAG_INIT, "%s\n%s", hw_infos.c_str(), infos_j.dump(2).c_str());
    OLED_CLEAR();
    OLED_PRINTF(
        "CPU: %iMHz, Flash: %iMiB\nDEVICE ID: %s\n",
        (int)ESP.getCpuFreqMHz(), (int)(ESP.getFlashChipSize() >> 20),
        node_info.device_id);

    if (sys.begin()) {
      ESP_LOGI(TAG_INIT, "LINK(OBD/GNSS?) coproc ver: %i, ", sys.devType);
    }

#if ENABLE_OBD
    obd.begin(sys.link);
#endif

#if ENABLE_MEMS
  if (!state.check(STATE_MEMS_READY)) {
    mems = init_MEMS(ENABLE_ORIENTATION);
    if (mems) {
      state.set(STATE_MEMS_READY);
    }
    ESP_LOGI(TAG_INIT, "MEMS: %s", mems? mems->name() : "NO");
  }  // !STATE_MEMS_READY
#endif

    state.set(STATE_WORKING);
    // initialize network and maintain connection
    subtask.create(telemetry, "telemetry", 2, 8192);

    ESP_LOGI(TAG_INIT, "<SETUP> completed");
    digitalWrite(PIN_LED, LOW);

    // initialize components
    initialize();

}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
    digitalWrite(PIN_LED, HIGH);
    initialize();
    digitalWrite(PIN_LED, LOW);
    return;
  }

  // collect and log data
  process();

  // check serial input for command
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      ESP_LOGV(TAG_PROC, "cmd(%s)...", serialCommand.c_str());
      if (serialCommand.length() > 0) {
        String result = executeCommand(serialCommand.c_str());

        bool is_err = result == "ERROR" || result == "INVALID CMD";
        ESP_LOG_LEVEL(
            is_err? ESP_LOG_ERROR : ESP_LOG_INFO,
            TAG_PROC,
            "cmd(%s): %s",
            serialCommand.c_str(),
            result.c_str());
        serialCommand = "";
      }
    } else if (serialCommand.length() < CMD_SERIAL_MAX_LEN) {
      serialCommand += c;
    }
  }

  digitalWrite(PIN_SENSOR2, digitalRead(PIN_SENSOR1));

  ESP_LOGD(TAG_INIT, "<loop> state: %X", state.m_state);
}
#endif // PIO_UNIT_TESTING
