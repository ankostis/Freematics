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

#include <esp_log.h>
#include <FreematicsPlus.h>
#include <httpd.h>
#include "config.h"
#include "telelogger.h"
#include "DeviceInfo.h"
#include "telemesh.h"
#include "teleclient.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_NET_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_STANDBY 0x100
#define STATE_GET_VEHICLE_INFO 0x200
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
char vin[18] = {0};
uint16_t dtc[6] = {0};
int16_t batteryVoltage = 0;
GPS_DATA* gd = 0;
char obfcmTest[128] = {0};

char devid[12] = {0};
char isoTime[26] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;
uint32_t lastObfcmTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;

bool serverSetup(IPAddress& ip);
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

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
#if ENABLE_MEMS
    processMEMS(0);
#endif
    delay(5);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

State state;

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
  int pins[] = {PIN_SENSOR1, PIN_SENSOR2};
  int pids[] = {PID_EXT_SENSOR1, PID_EXT_SENSOR2};
#if LOG_EXT_SENSORS == 1
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], digitalRead(pins[i]));
  }
#elif LOG_EXT_SENSORS == 2
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], analogRead(pins[i]));
  }
#endif
}
#endif

/*******************************************************************************
  HTTP API
*******************************************************************************/
#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, (float)batteryVoltage / 100);
    int n = snprintf(buf, bufsize, "{\"obfcm\":{\"obfcm\":\"%s\",\":[", obfcmTest);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, (unsigned int)(t - obdData[i].ts));
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if ENABLE_MEMS
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

int handlerControl(UrlHandlerParam* param)
{
    char *cmd = mwGetVarValue(param->pxVars, "cmd", 0);
    if (!cmd) return 0;
    String result = executeCommand(cmd);
    param->contentLength = snprintf(param->pucBuffer, param->bufSize,
        "{\"result\":\"%s\"}", result.c_str());
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
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

  Serial.print("[GPS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  if (gd->sat) {
    Serial.print(" SATS:");
    Serial.print(gd->sat);
  }
  Serial.print(" Course:");
  Serial.print(gd->heading);

  Serial.print(' ');
  Serial.println(isoTime);
  //Serial.println(gd->errors);
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
        Serial.print("Motion:");
        Serial.println(motion);
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
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
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
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
    // turn on buzzer at 2000Hz frequency
  sys.buzzer(2000);
  delay(100);
  // turn off buzzer
  sys.buzzer(0);

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
      Serial.println("GNSS:OK");
#if ENABLE_OLED
      oled.println("GNSS OK");
#endif
    } else {
      Serial.println("GNSS:NO");
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY | STATE_GET_VEHICLE_INFO);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("OBD:NO");
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

#if STORAGE != STORAGE_NONE
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
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      strncpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
    Serial.println("OBFCM:");
    int i = 0;
    obd.GetOBFCM(obfcmData);
    while (obfcmData[i].idx){
      Serial.println(obfcmData[i].value);
      i++;
    }

#if ENABLE_OLED
    oled.print("VIN:");
    oled.println(vin);
#endif
  }
#endif

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif

teleClient.notify(EVENT_PING); // Nestor
}

/*******************************************************************************
  Executing a remote command
*******************************************************************************/
String executeCommand(const char* cmd)
{
  String result;
  Serial.println(cmd);
  if (!strncmp(cmd, "LED", 3) && cmd[4]) {
    ledMode = (byte)atoi(cmd + 4);
    digitalWrite(PIN_LED, (ledMode == 2) ? HIGH : LOW);
    result = "OK";
  } else if (!strcmp(cmd, "REBOOT")) {
  #if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      logger.end();
      state.clear(STATE_STORAGE_READY);
    }
  #endif
    teleClient.shutdown();
    ESP.restart();
    // never reach here
  } else if (!strcmp(cmd, "STANDBY")) {
    state.clear(STATE_WORKING);
    result = "OK";
  } else if (!strcmp(cmd, "WAKEUP")) {
    state.clear(STATE_STANDBY);
    result = "OK";
  } else if (!strncmp(cmd, "SET", 3) && cmd[3]) {
    const char* subcmd = cmd + 4;
    if (!strncmp(subcmd, "SYNC", 4) && subcmd[4]) {
      syncInterval = atoi(subcmd + 4 + 1);
      result = "OK";
    } else {
      result = "ERROR";
    }
  } else if (!strcmp(cmd, "STATS")) {
    char buf[64];
    sprintf(buf, "TX:%u OBD:%u NET:%u", teleClient.txCount, timeoutsOBD, timeoutsNet);
    result = buf;
#if ENABLE_OBD
  } else if (!strncmp(cmd, "OBD", 3) && cmd[4]) {
    // send OBD command
    char buf[256];
    sprintf(buf, "%s\r", cmd + 4);
    if (obd.link && obd.link->sendCommand(buf, buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0) {
      Serial.println(buf);
      for (int n = 0; buf[n]; n++) {
        switch (buf[n]) {
        case '\r':
        case '\n':
          result += ' ';
          break;
        default:
          result += buf[n];
        }
      }
    } else {
      result = "ERROR";
    }
#endif
  } else {
    return "INVALID";
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
      Serial.println("ACK...");
      if (teleClient.notify(EVENT_ACK, buf)) {
        Serial.println("sent");
        break;
      }
    }
  } else {
    // previously executed command
    char buf[64];
    snprintf(buf, sizeof(buf), "TK=%u,DUP=1", token);
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (teleClient.notify(EVENT_ACK, buf)) {
        Serial.println("sent");
        break;
      }
    }
  }
  return true;
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char timestr[24];
  sprintf(timestr, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(timestr);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes");
  Serial.println();
#if ENABLE_OLED
  oled.setCursor(0, 2);
  oled.println(timestr);
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
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

    if (state.check(STATE_GET_VEHICLE_INFO)) {
      lastObfcmTime = millis();
      obd.GetOBFCM(obfcmData);
      state.clear(STATE_GET_VEHICLE_INFO);
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

    if (millis() - lastObfcmTime > 120000) {
      Serial.print("Two minutes since last OBFCM.");
      state.set(STATE_GET_VEHICLE_INFO);
    }

    if (obd.errors >= MAX_OBD_ERRORS) {
      if (!obd.init()) {
        Serial.println("ECU OFF");
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
    buffer->add(PID_BATTERY_VOLTAGE, (int)batteryVoltage);
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
    bufman.printStats();
    lastStatsTime = startTime;
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      Serial.print("[FILE] ");
      Serial.print(sizeKB);
      Serial.println("KB");
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
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended if OBD is not available
    if (!state.check(STATE_OBD_READY)) state.clear(STATE_WORKING);
  }
#endif
  long t = dataInterval - (millis() - startTime);
  if (t > 0 && t < dataInterval) delay(t);
}

bool initNetwork()
{
#if NET_DEVICE == NET_WIFI
#if ENABLE_OLED
  oled.print("Connecting WiFi...");
#endif
  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print("Joining ");
    Serial.println(WIFI_SSID);
    teleClient.net.begin(WIFI_SSID, WIFI_PASSWORD);
    if (teleClient.net.setup()) {
      state.set(STATE_NET_READY);
      String ip = teleClient.net.getIP();
      if (ip.length()) {
        state.set(STATE_NET_CONNECTED);
        Serial.print("WiFi IP:");
        Serial.println(ip);
#if ENABLE_OLED
        oled.println(ip);
#endif
        break;
      }
    } else {
      Serial.println("No WiFi");
    }
  }
#else
  // power on network module
  if (teleClient.net.begin(&sys)) {
    state.set(STATE_NET_READY);
  } else {
    Serial.println("CELL:NO");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
#if NET_DEVICE == SIM800 || NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
#if ENABLE_OLED
    oled.print(teleClient.net.deviceName());
    oled.println(" OK\r");
    oled.print("IMEI:");
    oled.println(teleClient.net.IMEI);
#endif
  Serial.print("CELL:");
  Serial.println(teleClient.net.deviceName());
  if (!teleClient.net.checkSIM(SIM_CARD_PIN)) {
    Serial.println("NO SIM CARD");
    return false;
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.net.IMEI);
  if (state.check(STATE_NET_READY) && !state.check(STATE_NET_CONNECTED)) {
    if (teleClient.net.setup(CELL_APN)) {
      String op = teleClient.net.getOperatorName();
      if (op.length()) {
        Serial.print("Operator:");
        Serial.println(op);
#if ENABLE_OLED
        oled.println(op);
#endif
      }

#if GNSS == GNSS_CELLULAR
      if (teleClient.net.setGPS(true)) {
        Serial.println("CELL GNSS:OK");
      }
#endif

      String ip = teleClient.net.getIP();
      if (ip.length()) {
        Serial.print("IP:");
        Serial.println(ip);
#if ENABLE_OLED
        oled.print("IP:");
        oled.println(ip);
#endif
      }
      rssi = teleClient.net.getSignal();
      if (rssi) {
        Serial.print("RSSI:");
        Serial.print(rssi);
        Serial.println("dBm");
#if ENABLE_OLED
        oled.print("RSSI:");
        oled.print(rssi);
        oled.println("dBm");
#endif
      }
      state.set(STATE_NET_CONNECTED);
    } else {
      char *p = strstr(teleClient.net.getBuffer(), "+CPSI:");
      if (p) {
        char *q = strchr(p, '\r');
        if (q) *q = 0;
        Serial.println(p + 7);
#if ENABLE_OLED
        oled.println(p + 7);
#endif
      } else {
        Serial.print(teleClient.net.getBuffer());
      }
    }
    timeoutsNet = 0;
  }
#else
  state.set(STATE_NET_CONNECTED);
#endif
#endif
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
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_NET_READY)) {
        teleClient.shutdown();
      }
      state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      teleClient.reset();
      bufman.purge();

#if GNSS == GNSS_STANDALONE
      if (state.check(STATE_GPS_READY)) {
        Serial.println("GNSS OFF");
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
          Serial.print("Ping...");
          bool success = teleClient.ping();
          Serial.println(success ? "OK" : "NO");
        }
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      }
      continue;
    }

    if (!state.check(STATE_NET_CONNECTED)) {
      if (!initNetwork() || !teleClient.connect()) {
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
        delay(10000);
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
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      buffer->purge();
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.tailer();
#endif
      //Serial.println(store.buffer());

      // start transmission
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
      if (teleClient.transmit(store.buffer(), store.length())) {
        // successfully sent
        connErrors = 0;
        showStats();
      } else {
        connErrors++;
        timeoutsNet++;
        printTimeoutStats();
      }
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);

      store.purge();

      teleClient.inbound();
      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("Instable connection");
        connErrors++;
        timeoutsNet++;
      }
      if (connErrors > MAX_CONN_ERRORS_RECONNECT) {
        teleClient.net.close();
        if (!teleClient.connect()) {
          teleClient.shutdown();
          state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        // device too hot, cool down by pause transmission
        Serial.println("Overheat");
        delay(5000);
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
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif
  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
  state.set(STATE_STANDBY | STATE_GET_VEHICLE_INFO);
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
#if ENABLE_MEMS
  uint32_t t;
  float v;
  uint32_t t_old = millis();
  float v_old = 999;
  float v_grad;

  calibrateMEMS();

  do {
    t = millis();
    v = (float)obd.getVoltage();

    v_grad = (v - v_old)/(t - t_old)*1000;
    Serial.print("Delta time: ");
    Serial.print((float)(t - t_old)/1000);
    Serial.print(", Voltage old: ");
    Serial.print(v_old);
    Serial.print(", Voltage new: ");
    Serial.print(v);
    Serial.print(", Gradient: ");
    Serial.println(v_grad);

    if (waitMotion(-1)) {
      Serial.println("Motion event detected.");
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
  Serial.println("Wakeup");

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
void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

#if CONFIG_MODE_TIMEOUT
void configMode()
{
  uint32_t t = millis();

  do {
    if (Serial.available()) {
      // enter config mode
      Serial.println("#CONFIG MODE#");
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
      Serial.println("#RESET#");
      delay(100);
      ESP.restart();
    }
  } while (millis() - t < CONFIG_MODE_TIMEOUT);
}
#endif

void setup()
{
    delay(500);

#if ENABLE_OLED
    oled.begin();
    oled.setFontSize(FONT_SIZE_SMALL);
#endif
    // initialize USB serial
    Serial.begin(115200);

    // Relevant only when ESP_IDF log-lib selected.
    esp_log_level_set("*", (esp_log_level_t)RUNTIME_ALL_TAGS_LOG_LEVEL);

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

   // generate unique device ID
    genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
    configMode();
#endif

#if LOG_EXT_SENSORS
    pinMode(PIN_SENSOR1, INPUT);
    pinMode(PIN_SENSOR2, INPUT);
#endif

    LogDeviceInfo(devid);

#if ENABLE_OLED
    oled.clear();
    oled.printf(
        "CPU: %iMHz, Flash: %iMiB\nDEVICE ID: %s\n",
        (int)ESP.getCpuFreqMHz(), (int)(ESP.getFlashChipSize() >> 20), devid);
#endif

    if (sys.begin()) {
      Serial.print("TYPE:");
      Serial.println(sys.devType);
    }

#if ENABLE_OBD
    obd.begin(sys.link);
#endif

#if ENABLE_MEMS
  if (!state.check(STATE_MEMS_READY)) {
    Serial.print("MEMS:");
    mems = new MPU9250;
    byte ret = mems->begin(ENABLE_ORIENTATION);
    if (ret) {
      state.set(STATE_MEMS_READY);
      Serial.println("MPU-9250");
    } else {
      mems->end();
      delete mems;
      mems = new ICM_20948_I2C;
      ret = mems->begin(ENABLE_ORIENTATION);
      if (ret) {
        state.set(STATE_MEMS_READY);
        Serial.println("ICM-20948");
      } else {
        Serial.println("NO");
      }
    }
  }
#endif

#if ENABLE_HTTPD
    IPAddress ip;
    if (serverSetup(ip)) {
      Serial.println("HTTPD:");
      Serial.println(ip);
#if ENABLE_OLED
      oled.println(ip);
#endif
    } else {
      Serial.println("HTTPD:NO");
    }
#endif

    state.set(STATE_WORKING);
    // initialize network and maintain connection
    subtask.create(telemetry, "telemetry", 2, 8192);
    // initialize components
    initialize();

    digitalWrite(PIN_LED, LOW);
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
      if (serialCommand.length() > 0) {
        String result = executeCommand(serialCommand.c_str());
        serialCommand = "";
        Serial.println(result);
      }
    } else if (serialCommand.length() < 32) {
      serialCommand += c;
    }
  }

  digitalWrite(26, digitalRead(34));
}
