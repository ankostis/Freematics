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
#include "telestore.h"
#include "teleclient.h"
#include "config.h"

bool processCommand(char* data);

extern node_info_t node_info;
extern int16_t rssi;
extern GPS_DATA* gd;
extern char isoTime[];

CBuffer::CBuffer(uint8_t* mem)
{
  m_data = mem;
  purge();
}

void CBuffer::add(uint16_t pid, uint8_t type, void* values, int bytes, uint8_t count)
{
  if (offset < BUFFER_LENGTH - sizeof(ELEMENT_HEAD) - bytes) {
    ELEMENT_HEAD hdr = {pid, type, count};
    *(ELEMENT_HEAD*)(m_data + offset) = hdr;
    offset += sizeof(ELEMENT_HEAD);
    memcpy(m_data + offset, values, bytes); 
    offset += bytes;
    total++;
  } else {
      ESP_LOGW(TAG_BUF, "FULL");
  }
}
void CBuffer::add(uint16_t pid, int32_t value)
{
  add(pid, ELEMENT_INT32, &value, sizeof(int32_t));
}
void CBuffer::add(uint16_t pid, float value)
{
  add(pid, ELEMENT_FLOAT, &value, sizeof(float));
}
void CBuffer::purge()
{
  state = BUFFER_STATE_EMPTY;
  timestamp = 0;
  offset = 0;
  total = 0;
}

void CBuffer::serialize(CStorage& store)
{
  uint16_t of = 0;
  for (int n = 0; n < total && of < offset; n++) {
    ELEMENT_HEAD* hdr = (ELEMENT_HEAD*)(m_data + of);
    of += sizeof(ELEMENT_HEAD);
    switch (hdr->type) {
    case ELEMENT_UINT8:
      store.log(hdr->pid, (uint8_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint8_t);
      break;
    case ELEMENT_UINT16:
      store.log(hdr->pid, (uint16_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint16_t);
      break;
    case ELEMENT_UINT32:
      store.log(hdr->pid, (uint32_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint32_t);
      break;
    case ELEMENT_INT32:
      store.log(hdr->pid, (int32_t*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(int32_t);
      break;
    case ELEMENT_FLOAT:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D1:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count, "%.1f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D2:
      store.log(hdr->pid, (float*)(m_data + of), hdr->count, "%.2f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    default:
      return;
    }
  }
}

void CBufferManager::init()
{
  total = BUFFER_SLOTS;
#if BOARD_HAS_PSRAM
    slots = (CBuffer**)heap_caps_malloc(BUFFER_SLOTS * sizeof(void*), MALLOC_CAP_SPIRAM);
#else
    slots = (CBuffer**)malloc(BUFFER_SLOTS * sizeof(void*));
#endif
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    void* mem;
#if BOARD_HAS_PSRAM
    mem = heap_caps_malloc(BUFFER_LENGTH, MALLOC_CAP_SPIRAM);
#else
    mem = malloc(BUFFER_LENGTH);
#endif
    if (!mem) {
      ESP_LOGW(TAG_BUF, "OUT OF RAM");
      total = n;
      break;
    }
    slots[n] = new CBuffer((uint8_t*)mem);
  }
  // TODO: `(const -> variable) node_info.nslots = CBufferManager::total;`
  assert(total > 0);
}

void CBufferManager::purge()
{
  for (int n = 0; n < total; n++) slots[n]->purge();
  ESP_LOGI(TAG_BUF, "Purged %u buffers", total);
}

CBuffer* CBufferManager::getFree()
{
  if (last) {
    CBuffer* slot = last;
    last = 0;
    if (slot->state == BUFFER_STATE_EMPTY) return slot;
  }
  uint32_t ts = 0xffffffff;
  int m = 0;
  // search for free slot, if none, mark the oldest one
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_EMPTY) {
      return slots[n];
    } else if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  // dispose oldest data when buffer is full
  while (slots[m]->state == BUFFER_STATE_LOCKED) delay(1);
  slots[m]->purge();
  return slots[m];
}

CBuffer* CBufferManager::getOldest()
{
  uint32_t ts = 0xffffffff;
  int m = -1;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

CBuffer* CBufferManager::getNewest()
{
  uint32_t ts = 0;
  int m = -1;
  for (int n = 0; n < total; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp > ts) {
      m = n;
      ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

void CBufferManager::free(CBuffer* slot)
{
  slot->purge();
  last = slot;  
}

void CBufferManager::showCacheStats(uint16_t state)
{
  int bytes = 0;
  int count = 0;
  int samples = 0;
  for (int n = 0; n < total; n++) {
      if (slots[n]->state != BUFFER_STATE_FILLED) continue;
      bytes += slots[n]->offset;
      samples += slots[n]->total;
      count++;
      ESP_LOGV(TAG_BUF, "buf: %i: count: %i, offset: %i", n,
              slots[n]->total, slots[n]->offset);
  }
  if (count) {
      // TODO: Calc used RAM in bytes and with `ESP.getFreeHeap()` in `showCacheStats()`.
      constexpr const uint RAM_SIZE_KiB = 320;
      uint ram_used = RAM_SIZE_KiB - (ESP.getFreeHeap() >> 10);
      ESP_LOG_LEVEL(
              (count > 1? ESP_LOG_INFO : ESP_LOG_DEBUG),
              TAG_BUF,
              "PIDs: %u(%.2f b/PID)"
              ", slots: %u/%u(%.2f%%)"
              ", filled: %u/%u bytes (%.1f%%)"
              ", RAM: %u/%u KiB(%.1f%%)"
              ", state: %X",
              samples, samples ? ((float)bytes / samples) : 0.0,
              count, total, 100.0 * count / total,
              bytes, total * BUFFER_LENGTH,
              100.0 * bytes / (total * BUFFER_LENGTH),
              ram_used, RAM_SIZE_KiB, 100.0 * ram_used / RAM_SIZE_KiB,
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
      return cell.open(0, 0);  // Preserve existing host/port.
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
  for (byte n = 0; n < 3 && !success; n++) {
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
    if (success) {
      if ((success = notify(EVENT_PING))) break;
#if ENABLE_WIFI
      if (wifi.connected())
      {
        wifi.close();
      }
      else
#endif
      {
        cell.close();
      }
      delay(1000);
    }
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
    ESP_LOGD(TAG_UDP, "Inbound %ib: %s", len, data);
    rxBytes += len;
    if (!verifyChecksum(data)) {
      err = "bad checksum";
      ESP_LOGE(TAG_UDP, "Inbound %ib Checksum mismatch: %s", len, data);
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
  if (wifi.connected()) {
    wifi.end();
    ESP_LOGI(TAG_AWIFI, "<SHUTDOWN>");
    return;
  }
#endif
  cell.end();
  ESP_LOGI(TAG_ACELL, "<SHUTDOWN> %s", cell.deviceName());
}

bool TeleClientHTTP::notify(byte event, const char* payload)
{
  char path[256];
  snprintf(path, sizeof(path), "%s/notify/%s?EV=%u&SSI=%d&VIN=%s", 
      node_info.srv_path, node_info.device_id.c_str(), (uint)event, (int)rssi, 
      (const char *)node_info.vin);
  if (event == EVENT_LOGOUT) login = false;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    return wifi.send(METHOD_GET, path) && wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1) 
        && wifi.code() == 200;
  }
  else
#endif
  {
    return cell.send(METHOD_GET, node_info.srv_host, node_info.srv_port, path) 
        && cell.receive() && cell.code() == 200;
  }
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.state() != HTTP_CONNECTED) || cell.state() != HTTP_CONNECTED) {
#else
  if (cell.state() != HTTP_CONNECTED) {
#endif
    // reconnect if disconnected
    if (!connect(true)) {
      return false;
    }
  }

  const char *devid = node_info.device_id.c_str();
  char path[256];
  bool success = false;
  int len;
#if SERVER_PROTOCOL == PROTOCOL_HTTPS_GET
  auto srv_path = node_info.srv_path;
  if (gd && gd->ts) {
    len = snprintf(path, sizeof(path), 
        "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      srv_path, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(path, sizeof(path), "%s/push?id=%s", srv_path, devid);
  }
  success = cell.send(METHOD_GET, node_info.srv_host, node_info.srv_port, path);
#else
  len = snprintf(path, sizeof(path), "%s/post/%s", SERVER_PATH, devid);
#if ENABLE_WIFI
  if (wifi.connected()) {
    ESP_LOGD(TAG_AWIFI, "HTTP_POST %ib -> %s: %s", packetSize, host2log, path);
    success = wifi.send(METHOD_POST, path, packetBuffer, packetSize);
  }
  else
#endif
  {
    ESP_LOGD(TAG_ACELL, "HTTP_POST %ib -> %s: %s", packetSize, host2log, path);
    success = cell.send(METHOD_POST, node_info.srv_host, node_info.srv_port, path, packetBuffer, packetSize);
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
    content = cell.receive(&recvBytes, HTTP_CONN_TIMEOUT);
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
  if (!quick) {
#if ENABLE_WIFI
    if (!wifi.connected()) cell.init();
#else
    cell.init();
#endif
  } else {
#if ENABLE_WIFI
    if (!wifi.connected()) cell.close();
#else
    cell.close();
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
  if (wifi.connected()) {
    wifi.end();
    ESP_LOGI(TAG_AWIFI, "<SHUTDOWN> %s", wifi.deviceName());
    return;
  }
#endif
  cell.close();
  cell.end();
  ESP_LOGI(TAG_ACELL, "<SHUTDOWN> %s", cell.deviceName());
}
