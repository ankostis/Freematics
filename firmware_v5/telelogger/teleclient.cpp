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

#include <FreematicsPlus.h>
#include <NodeInfo.h>
#include "telelogger.h"
#include "telemesh.h"
#include "teleclient.h"

bool processCommand(char* data);

extern node_info_t node_info;
extern int16_t rssi;
extern GPS_DATA* gd;
extern char isoTime[];

CBuffer::CBuffer()
{
  purge();
}

void CBuffer::add(uint16_t pid, int value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(int)) {
    setType(ELEMENT_INT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(int*)(data + offset) = value;
    offset += sizeof(int);
    count++;
  } else {
    ESP_LOGW(TAG_BUF, "FULL");
  }
}
void CBuffer::add(uint16_t pid, uint32_t value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(uint32_t)) {
    setType(ELEMENT_UINT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(uint32_t*)(data + offset) = value;
    offset += sizeof(uint32_t);
    count++;
  } else {
    ESP_LOGW(TAG_BUF, "FULL");
  }
}
void CBuffer::add(uint16_t pid, float value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(float)) {
    setType(ELEMENT_FLOAT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(float*)(data + offset) = value;
    offset += sizeof(float);
    count++;
  } else {
    ESP_LOGW(TAG_BUF, "FULL");
  }
}
void CBuffer::add(uint16_t pid, float value[])
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) + sizeof(float) * 3) {
    setType(ELEMENT_FLOATX3);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    memcpy(data + offset, value, sizeof(float) * 3);
    offset += sizeof(float) * 3;
    count++;
  } else {
      ESP_LOGW(TAG_BUF, "FULL");
  }
}
void CBuffer::purge()
{
  state = BUFFER_STATE_EMPTY;
  timestamp = 0;
  offset = 0;
  count = 0;
  memset(types, 0, sizeof(types));
}

void CBuffer::setType(uint32_t dataType)
{
  types[count / 16] |= (dataType << ((count % 16) * 2));
}

void CBuffer::serialize(CStorage& store)
{
  int of = 0;
  for (int n = 0; n < count; n++) {
    uint16_t pid = *(uint16_t*)(data + of);
    of += sizeof(uint16_t);
    switch ((types[n / 16] >> ((n % 16) * 2)) & 0x3) {
    case ELEMENT_INT:
      {
        int value = *(int*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_UINT:
      {
        uint32_t value = *(uint32_t*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_FLOAT:
      {
        float value = *(float*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_FLOATX3:
      {
        float value[3];
        memcpy(value, data + of, sizeof(value));
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    }
  }
}

bool TeleClientUDP::verifyChecksum(char* data)
{
  uint8_t sum = 0;
  char *s = strrchr(data, '*');
  if (!s) return false;
  for (char *p = data; p < s; p++) sum += *p;
  if (hex2uint8(s + 1) == sum) {
    *s = 0;
    return true;
  }
  return false;
}

bool TeleClientUDP::notify(byte event, const char* payload)
{
  const char *devid = node_info.device_id.c_str();
  const char *vin = node_info.vin;
  char buf[48];
  CStorageRAM netbuf;
  netbuf.init(128);
  netbuf.header(devid);
  netbuf.dispatch(buf, sprintf(buf, "EV=%X", (unsigned int)event));
  netbuf.dispatch(buf, sprintf(buf, "TS=%lu", millis()));
  netbuf.dispatch(buf, sprintf(buf, "ID=%s", devid));
  if (rssi) {
    netbuf.dispatch(buf, sprintf(buf, "SSI=%d", (int)rssi));
  }
  if (vin[0]) {
    netbuf.dispatch(buf, sprintf(buf, "VIN=%s", vin));
  }
  if (payload) {
    netbuf.dispatch(payload, strlen(payload));
  }
  netbuf.tailer();
  ESP_LOGD(TAG_NET, "TeleClientUDP notify: |%s|", netbuf.buffer());
  for (byte attempts = 0; attempts < 3; attempts++) {
    // send notification datagram
    ESP_LOGV(TAG_NET, "notify x%i...", attempts);
    if (!net.send(netbuf.buffer(), netbuf.length()))
    {
      // error sending data
      break;
    }
#if NET_DEVICE != NET_SERIAL
    if (event == EVENT_ACK) return true; // no reply for ACK
    char *data = 0;
    // receive reply
    uint32_t t = millis();
    do {
      if ((data = net.receive())) break;
      // no reply yet
      delay(100);
    } while (millis() - t < node_info.net_recv_timeout_ms);
    if (!data) {
      ESP_LOGW(TAG_NET, "RECV timeout for event(%i)", event);
      continue;
    }
    // verify checksum
    if (!verifyChecksum(data)) {
      ESP_LOGE(TAG_NET, "Checksum mismatch: %s", data);
      continue;
    }
    char pattern[16];
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      ESP_LOGE(TAG_NET, "Invalid reply: %s, expected event: %i", data, event);
      continue;
    }
    if (event == EVENT_LOGIN) {
      // extract info from server response
      char *p = strstr(data, "TM=");
      if (p) {
        // set local time from server
        unsigned long tm = atol(p + 3);
        struct timeval tv = { .tv_sec = (time_t)tm, .tv_usec = 0 };
        settimeofday(&tv, NULL);
      }
      p = strstr(data, "SN=");
      if (p) {
        char *q = strchr(p, ',');
        if (q) *q = 0;
      }
      feedid = hex2uint16(data);
      login = true;
    } else if (event == EVENT_LOGOUT) {
      login = false;
    }
#endif
    // success
    return true;
  }
  return false;
}

bool TeleClientUDP::connect()
{
  byte event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  bool success = false;
  // connect to telematics server
  for (byte attempts = 0; attempts < node_info.net_retries; attempts++) {
    ESP_LOGD(TAG_NET, "Connecting to %s:%i...", host2log, port2log);
    if (!net.open(node_info.srv_host, node_info.srv_port)) {
      ESP_LOGW(TAG_NET, "Fail no-%i to connect to %s:%i, wait %isec...",
               attempts, host2log, port2log, node_info.net_udp_reconnect_delay_ms);
      delay(node_info.net_udp_reconnect_delay_ms);
      continue;
    }
    // log in or reconnect to Freematics Hub
    success = notify(event);
    ESP_LOG_LEVEL(
      (success? ESP_LOG_INFO : ESP_LOG_ERROR),
      TAG_NET,
      "%s to %s:%i (attempt no-%i) %s",
      (event == EVENT_LOGIN ? "LOGIN" : "RECONNECT"),
      host2log,
      port2log,
      attempts,
      success? "OK" : "FAILED!");

    if (success) break;

    net.close();
  }  // connect attempts loop

  startTime = millis();
  if (success) {
    lastSyncTime = startTime;
  }
  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (byte n = 0; n < 2 && !success; n++) {
    success = net.open(node_info.srv_host, node_info.srv_port);
    if (success) success = notify(EVENT_PING);
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  bool success = false;
  // transmit data
  if (net.send(packetBuffer, packetSize)) {
    txBytes += packetSize;
    txCount++;
    success = true;
  }
  return success;
}

bool TeleClientUDP::inbound()
{
  // check incoming datagram
  const char *err;
  do {
    int len = 0;
    char *data = net.receive(&len, 0);
    if (!data) {
      err = "timeout";
      break;
    }
    data[len] = 0;
    rxBytes += len;
    if (!verifyChecksum(data)) {
      err = "bad checksum";
      break;
    }
    char *p = strstr(data, "EV=");
    if (!p) {
      err = "no event";
      break;
    }

    // By now server-connection assumed OK, mark sync-time.
    lastSyncTime = millis();

    int eventID = atoi(p + 3);
    switch (eventID) {
      case EVENT_COMMAND:
        processCommand(data);
        break;

      case EVENT_SYNC: {
        uint16_t id = hex2uint16(data);
        if (id && id != feedid) {
          feedid = id;
          ESP_LOGI(TAG_NET, "FEED ID: %i", feedid);
        }
      } break;

      default:
        ESP_LOGW(TAG_NET, "Unknown inbound event: %i", eventID);
    }  // switch eventID

    return true;

  } while (0);

  ESP_LOGW(TAG_NET, "Inbound error: %s", err);
  auto timeout = node_info.srv_sync_timeout_ms;
  auto ok = timeout == 0 || (millis() - lastSyncTime) < timeout;

  return ok;
}

void TeleClientUDP::shutdown()
{
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
    net.close();
  }
  net.end();
  ESP_LOGI(TAG_NET, "<SHUTDOWN> %s", net.deviceName());
}

bool TeleClientHTTP::notify(byte event, const char* payload)
{
  char url[256];
  snprintf(url, sizeof(url), "%s/notify/%s?EV=%u&SSI=%d&VIN=%s", node_info.srv_path,
           node_info.device_id.c_str(), (uint)event, (int)rssi,
           (const char *)node_info.vin);
  if (event == EVENT_LOGOUT) login = false;
  return net.send(METHOD_GET, url, true) && net.receive();
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  if (net.state() != HTTP_CONNECTED) {
    // reconnect if disconnected
    if (!connect()) {
      return false;
    }
  }

  const char *devid = node_info.device_id.c_str();
  char url[256];
  bool success = false;
  int len;
  auto srv_path = node_info.srv_path;
#if SERVER_METHOD == PROTOCOL_METHOD_GET
  if (gd && gd->ts) {
    len = snprintf(url, sizeof(url), "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      srv_path, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(url, sizeof(url), "%s/push?id=%s", srv_path, devid);
  }
  success = net.send(METHOD_GET, url, true);
#else
  len = snprintf(url, sizeof(url), "%s/post/%s", srv_path, devid);
  ESP_LOGD(TAG_NET,
      "TeleClientHTTP sending %i bytes to URL: %s",
      packetSize,
#if HIDE_SECRETS_IN_LOGS
      host2log
#else
      url
#endif
  );
  success = net.send(METHOD_POST, url, true, packetBuffer, packetSize);
  len += packetSize;
#endif
  if (!success) {
    ESP_LOGE(TAG_NET, "Transmit failed. Closing net");
    net.close();
    return false;
  } else {
    txBytes += len;
    txCount++;
  }

  // check response
  int bytes = 0;
  char* response = net.receive(&bytes);
  if (!response) {
    // close connection on receiving timeout
    ESP_LOGE(TAG_NET, "No HTTP response.  Closing net.");
    net.close();
    return false;
  }
  ESP_LOGD(TAG_NET, "tx-reply: %s", response);
  if (net.code == 200) {
    lastSyncTime = millis();
    rxBytes += bytes;
  }
  return true;
}

bool TeleClientHTTP::connect()
{
  if (!started) {
    started = net.open();
  }

  // connect to HTTP server
  bool success = false;
  for (byte attempts = 0; !success && attempts < node_info.net_retries;
       attempts++) {
    success = net.open(node_info.srv_host, node_info.srv_port);
    if (!success) {
      net.close();
      delay(1000);
    }
  }
  if (!success) {
    ESP_LOGE(TAG_NET, "Error connecting to server");
    return false;
  }
  if (!login) {
    // log in or reconnect to Freematics Hub
    if ((login = notify(EVENT_LOGIN))) {
      lastSyncTime = millis();
    }
    ESP_LOG_LEVEL(
      (login? ESP_LOG_INFO : ESP_LOG_ERROR),
      TAG_NET,
      "LOGIN to %s:%i %s",
      host2log,
      port2log,
      login? "OK" : "FAILED!");
  } // was not logged-in.
  return true;
}

bool TeleClientHTTP::ping()
{
  return connect();
}

void TeleClientHTTP::shutdown()
{
  Serial.print(net.deviceName());
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
  }
  net.close();
  net.end();
  Serial.println(" OFF");
  started = false;
  ESP_LOGI(TAG_NET, "<SHUTDOWN> %s", net.deviceName());
}
