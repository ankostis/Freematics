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
#if BOARD_HAS_PSRAM
  data = (uint8_t*)heap_caps_malloc(BUFFER_LENGTH, MALLOC_CAP_SPIRAM);
  types = (uint32_t*)heap_caps_malloc((BUFFER_LENGTH / (sizeof(uint16_t) + sizeof(int)) + 15) / 16, MALLOC_CAP_SPIRAM);
#else
  data = (uint8_t*)malloc(BUFFER_LENGTH);
  types = (uint32_t*)malloc((BUFFER_LENGTH / (sizeof(uint16_t) + sizeof(int)) + 15) / 16);
#endif
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

void CBufferManager::init()
{
  for (int n = 0; n < BUFFER_SLOTS; n++) {
      buffers[n] = new CBuffer();
  }
}

void CBufferManager::purge()
{
  int purged = 0;
  for (auto *buf: buffers) {
      if (buf->count) purged++;
      buf->purge();
  }
  ESP_LOGI(TAG_BUF, "Purged %u buffers", purged);

}

CBuffer* CBufferManager::get(byte state)
{
    for (int n = 0; n < BUFFER_SLOTS; n++) {
        if (buffers[n]->state == state) return buffers[n];
    }
    return 0;
}

CBuffer* CBufferManager::getOldest()
{
  uint32_t ts = 0xffffffff;
  int m = -1;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
      if (buffers[n]->state == BUFFER_STATE_FILLED && buffers[n]->timestamp < ts) {
          m = n;
          ts = buffers[n]->timestamp;
      }
  }
  return m >= 0 ? buffers[m] : 0;
}

CBuffer* CBufferManager::getNewest()
{
  uint32_t ts = 0;
  int m = -1;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
      if (buffers[n]->state == BUFFER_STATE_FILLED && buffers[n]->timestamp > ts) {
          m = n;
          ts = buffers[n]->timestamp;
      }
  }
  return m >= 0 ? buffers[m] : 0;
}

void CBufferManager::showCacheStats(uint16_t state)
{
  int bytes = 0;
  int slots = 0;
  int samples = 0;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
      if (buffers[n]->state != BUFFER_STATE_FILLED) continue;
      bytes += buffers[n]->offset;
      samples += buffers[n]->count;
      slots++;
      ESP_LOGV(TAG_BUF, "buf: %i: count: %i, offset: %i", n,
              buffers[n]->count, buffers[n]->offset);
  }
  if (slots) {
      constexpr const uint RAM_SIZE_KiB = 320;
      uint ram_used = RAM_SIZE_KiB - (ESP.getFreeHeap() >> 10);
      ESP_LOG_LEVEL(
              (slots > 1? ESP_LOG_INFO : ESP_LOG_DEBUG),
              TAG_BUF,
              "PIDs: %u(%u b/PID)"
              ", slots: %u/%u(%u%%)"
              ", filled: %u/%u bytes (%u%%)"
              ", RAM: %u/%u KiB(%u%%)"
              ", state: %X",
              samples, samples ? bytes / samples : 0,
              slots, BUFFER_SLOTS, 100 * slots / BUFFER_SLOTS,
              bytes, BUFFER_SLOTS * BUFFER_LENGTH,
              100 * bytes / (BUFFER_SLOTS * BUFFER_LENGTH),
              ram_used, RAM_SIZE_KiB, 100 * ram_used / 320,
              state);
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
  char cache[128];
  CStorageRAM netbuf;
  netbuf.init(cache, 128);
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
  ESP_LOGD(TAG_UDP, "notify: |%s|", netbuf.buffer());
  for (byte attempts = 0; attempts < 3; attempts++) {
    // send notification datagram
    ESP_LOGV(TAG_UDP, "notify x%i...", attempts);
#if ENABLE_WIFI
    if (wifi.connected())
    {
      if (!wifi.send(netbuf.buffer(), netbuf.length())) break;
    }
    else
#endif
    {
      if (!cell.send(netbuf.buffer(), netbuf.length())) break;
    }
    if (event == EVENT_ACK) return true; // no reply for ACK
    char *data = 0;
    int bytesRecv = 0;
    // receive reply
#if ENABLE_WIFI
    if (wifi.connected())
    {
      data = cell.getBuffer();
      bytesRecv = wifi.receive(data, RECV_BUF_SIZE - 1);
      if (bytesRecv > 0) {
        data[bytesRecv] = 0;
      }
    }
    else
#endif
    {
      data = cell.receive(&bytesRecv);
    }
    if (!data || bytesRecv == 0) {
      ESP_LOGW(TAG_UDP, "RECV timeout for event(%i)", event);
      continue;
    }
    rxBytes += bytesRecv;
    // verify checksum
    if (!verifyChecksum(data)) {
      ESP_LOGE(TAG_UDP, "RECV checksum mismatch: %s", data);
      continue;
    }
    char pattern[16];
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      ESP_LOGE(TAG_UDP, "RECV invalid reply: %s, expected event: %i", data, event);
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
    // success
    return true;
  }
  return false;
}

bool TeleClientUDP::connect(bool quick)
{
  const byte event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  const char *event_name = login ? "LOGIN" : "RECONNECT";
  const char *srv_host = node_info.srv_host;
  const uint16_t srv_port = node_info.srv_port;
  const uint16_t net_retries = node_info.net_retries;
  const uint16_t delay_ms = node_info.net_udp_reconnect_delay_ms;
  bool success = false;

#if ENABLE_WIFI
  if (wifi.connected())
  {
    if (quick) return wifi.open(srv_host, srv_port);
  }
  else
#endif
  {
    cell.close();
    if (quick) {
      return cell.open(0, 0);
    }
  }

  packets = 0;

  // connect to telematics server
  for (byte attempts = 0; attempts < net_retries; attempts++) {
    ESP_LOGD(TAG_UDP, "%s(%s:%i)...", event_name, host2log, port2log);
#if ENABLE_WIFI
    if (wifi.connected())
    {
      if (!wifi.open(srv_host, srv_port)) {
        ESP_LOGW(TAG_AWIFI, "WIFI fail no-%i to connect to %s:%i, wait %isec...",
                attempts, host2log, port2log, delay_ms);
        delay(delay_ms);
        continue;
      }
    }
    else
#endif
    {
      if (!cell.open(srv_host, srv_port)) {
        if (!cell.check()) break;
          ESP_LOGW(TAG_UDP, "CELL Fail no-%i to connect to %s:%i, wait %isec...",
               attempts, host2log, port2log, 3 * delay_ms);
        delay(3 * delay_ms);
        continue;
      }
    }
    // log in or reconnect to Freematics Hub
    success = notify(event);
    ESP_LOG_LEVEL(
      (success? ESP_LOG_INFO : ESP_LOG_ERROR),
      TAG_UDP,
      "%s to %s:%i (attempt no-%i) %s",
      event_name,
      host2log,
      port2log,
      attempts,
      success? "OK" : "FAILED!");

    if (success) {
        lastSyncTime = millis();
        if (event == EVENT_LOGIN) startTime = lastSyncTime;

        break;
    }

#if ENABLE_WIFI
    if (wifi.connected())
    {
      wifi.close();
    }
    else
#endif
    {
      if (!cell.check()) break;
      cell.close();
    }
  }

  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (byte n = 0; n < 2 && !success; n++) {
#if ENABLE_WIFI
    if (wifi.connected())
    {
      success = wifi.open(node_info.srv_host, node_info.srv_port);
    }
    else
#endif
    {
      success = cell.open(node_info.srv_host, node_info.srv_port);
    }
    if (success) success = notify(EVENT_PING);
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  // transmit data via wifi
  if (wifi.connected()) {
    if (wifi.send(packetBuffer, packetSize)) {
      txBytes += packetSize;
      txCount++;
      ESP_LOGD(TAG_AWIFI, "TX %u bytes", packetSize);
      return true;
    }
    return false;
  }
#endif

  // transmit data via cellular
  if (++packets >= 64) {
    cell.close();
    cell.open(0, 0);
    packets = 0;
  }
  ESP_LOGD(TAG_ACELL, "TX %u bytes", packetSize);
  if (cell.send(packetBuffer, packetSize)) {
    txBytes += packetSize;
    txCount++;
    return true;
  }
  return false;
}

void TeleClientUDP::inbound()
{
  // check incoming datagram
  const char *err;
  do {
    int len = 0;
    char *data = 0;
#if ENABLE_WIFI
    if (wifi.connected())
    {
      data = cell.getBuffer();
      len = wifi.receive(data, RECV_BUF_SIZE - 1, 10);
    }
    else
#endif
    {
      data = cell.receive(&len, 50);
    }
    if (!data) {
      err = "timeout";
      break;
    }
    data[len] = 0;
    ESP_LOGD(TAG_UDP, "Inbound: %s", data);
    rxBytes += len;
    if (!verifyChecksum(data)) {
      err = "bad checksum";
      ESP_LOGE(TAG_UDP, "Inbound Checksum mismatch: %s", data);
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

      case EVENT_SYNC:
        {
          uint16_t id = hex2uint16(data);
          if (id && id != feedid) {
            feedid = id;
            ESP_LOGI(TAG_UDP, "FEED ID: %i", feedid);
          }
        }
        break;

      default:
        ESP_LOGW(TAG_UDP, "Unknown inbound event: %i", eventID);
    }  // switch eventID

    return;

  } while (0);

  ESP_LOGW(TAG_UDP, "Inbound error: %s", err);
}

void TeleClientUDP::shutdown()
{
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
    ESP_LOGI(TAG_UDP, "<LOGOUT>");
  }
#if ENABLE_WIFI
  wifi.end();
  ESP_LOGI(TAG_AWIFI, "<SHUTDOWN>");
#endif
  cell.end();
  ESP_LOGI(TAG_ACELL, "<SHUTDOWN> %s", cell.deviceName());
}

bool TeleClientHTTP::notify(byte event, const char* payload)
{
  char url[256];
  snprintf(url, sizeof(url), "%s/notify/%s?EV=%u&SSI=%d&VIN=%s", node_info.srv_path,
           node_info.device_id.c_str(), (uint)event, (int)rssi,
           (const char *)node_info.vin);
  if (event == EVENT_LOGOUT) login = false;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    return wifi.send(METHOD_GET, url, true) && wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1) && wifi.code() == 200;
  }
  else
#endif
  {
    return cell.send(METHOD_GET, url, true) && cell.receive() && cell.code() == 200;
  }
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  if (wifi.connected() && (wifi.state() != HTTP_CONNECTED || cell.state() != HTTP_CONNECTED)) {
#else
  if (cell.state() != HTTP_CONNECTED) {
#endif
    // reconnect if disconnected
    if (!connect(true)) {
      return false;
    }
  }

  const char *devid = node_info.device_id.c_str();
  char url[256];
  bool success = false;
  int len;
#if SERVER_METHOD == PROTOCOL_METHOD_GET
  auto srv_path = node_info.srv_path;
  if (gd && gd->ts) {
    len = snprintf(url, sizeof(url), "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      srv_path, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(url, sizeof(url), "%s/push?id=%s", srv_path, devid);
  }
  success = cell.send(METHOD_GET, url, true);
#else
  len = snprintf(url, sizeof(url), "%s/post/%s", SERVER_PATH, devid);
#if ENABLE_WIFI
  if (wifi.connected()) {
    ESP_LOGD(TAG_AWIFI, "Transmit %i bytes: %s", packetSize, host2log);
    success = wifi.send(METHOD_POST, url, true, packetBuffer, packetSize);
  }
  else
#endif
  {
    ESP_LOGD(TAG_ACELL, "Transmit %i bytes: %s", packetSize, host2log);
    success = cell.send(METHOD_POST, url, true, packetBuffer, packetSize);
  }
  len += packetSize;
#endif
  if (!success) {
    ESP_LOGE(TAG_HTTP, "Transmit failed.");
    return false;
  } else {
    txBytes += len;
    txCount++;
  }

  // check response
  int recvBytes = 0;
  char* content = 0;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    content = wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1, &recvBytes);
  }
  else
#endif
  {
    content = cell.receive(&recvBytes);
  }
  if (!content) {
    // close connection on receiving timeout
    ESP_LOGE(TAG_HTTP, "No response");
    return false;
  }
  ESP_LOGD(TAG_HTTP, "tx-reply: %s", content);
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.code() == 200) || cell.code() == 200) {
#else
  if (cell.code() == 200) {
#endif
    // successful
    lastSyncTime = millis();
    rxBytes += recvBytes;
  }
  return true;
}

bool TeleClientHTTP::connect(bool quick)
{
  cell.close();
  if (!quick) {
#if ENABLE_WIFI
    if (!wifi.connected()) cell.init();
#else
    cell.init();
#endif
  }

  // connect to HTTP server
  bool success = false;

#if ENABLE_WIFI
  if (wifi.connected())
    success = wifi.open(node_info.srv_host, node_info.srv_port);
#endif
  if (!success) {
    for (byte attempts = 0; !success && attempts < node_info.net_retries;
        attempts++) {
      success = cell.open(node_info.srv_host, node_info.srv_port);
      if (!success) {
        if (!cell.check()) break;
        cell.close();
        cell.init();
      }
    }
  }
  if (!success) {
    ESP_LOGE(TAG_HTTP, "Error connecting to server");
    return false;
  }
  if (quick) return true;
  if (!login) {
    // log in or reconnect to Freematics Hub
    if ((login = notify(EVENT_LOGIN))) {
      lastSyncTime = millis();
    }
    ESP_LOG_LEVEL(
      (login? ESP_LOG_INFO : ESP_LOG_ERROR),
      TAG_HTTP,
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
  if (login) {
    ESP_LOGI(TAG_HTTP, "<LOGOUT>");
    notify(EVENT_LOGOUT);
    login = false;
  }
#if ENABLE_WIFI
  wifi.end();
  ESP_LOGI(TAG_AWIFI, "<SHUTDOWN> %s", wifi.deviceName());
#endif
  cell.close();
  cell.end();
  ESP_LOGI(TAG_ACELL, "<SHUTDOWN> %s", cell.deviceName());
}
