#include "config.h"

// TODO: `teleclient.h' is a malstructured header-file, cannot include others.

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

#define BUFFER_STATE_EMPTY 0
#define BUFFER_STATE_FILLING 1
#define BUFFER_STATE_FILLED 2
#define BUFFER_STATE_LOCKED 3

#define ELEMENT_INT 0
#define ELEMENT_UINT 1
#define ELEMENT_FLOAT 2
#define ELEMENT_FLOATX3 3

// ESP_IDF logging tag used
inline constexpr const char TAG_BUF[] = "BUF";
inline constexpr const char TAG_NET[] = "NET";

class CBuffer
{
public:
    CBuffer();
    void add(uint16_t pid, int value);
    void add(uint16_t pid, uint32_t value);
    void add(uint16_t pid, float value);
    void add(uint16_t pid, float value[]);
    void purge();
    void serialize(CStorage& store);
    uint32_t timestamp;
    int offset;
    uint16_t count;
    uint8_t state;
private:
    void setType(uint32_t dataType);
    uint8_t data[BUFFER_LENGTH];
    uint32_t types[(BUFFER_LENGTH / (sizeof(uint16_t) + sizeof(int)) + 15) / 16];
};

class CBufferManager
{
public:
    CBufferManager()
    {
        for (int n = 0; n < BUFFER_SLOTS; n++) buffers[n] = new CBuffer;
    }
    void purge()
    {
        int purged = 0;
        for (auto *buf: buffers) {
            if (buf->count) purged++;
            buf->purge();
        }
        ESP_LOGI(TAG_BUF, "Purged %u buffers", purged);
    }
    CBuffer* get(byte state = BUFFER_STATE_EMPTY)
    {
        for (int n = 0; n < BUFFER_SLOTS; n++) {
            if (buffers[n]->state == state) return buffers[n];
        }
        return 0;
    }
    CBuffer* getOldest()
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
    CBuffer* getNewest()
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
    void showCacheStats(uint16_t state)
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
    CBuffer* buffers[BUFFER_SLOTS];
};

class TeleClient
{
public:
    virtual void reset()
    {
        txCount = 0;
        txBytes = 0;
        rxBytes = 0;
        login = false;
    }
    virtual bool notify(byte event, const char* payload = 0) { return true; }
    virtual bool connect() { return true; }
    virtual bool transmit(const char* packetBuffer, unsigned int packetSize)  { return true; }
    virtual void inbound() {}
    void showNetStats(char *timestr, uint16_t state) {
        uint32_t t = millis() - startTime;
        sprintf(timestr,
                "%02u:%02u.%c",
                t / 60000,
                (t % 60000) / 1000,
                (t % 1000) / 100 + '0');
        ESP_LOGI(TAG_NET,
            "%s: packet #%i, Tx: %.2fKiB, Rx: %ib, login: %i, feedid: %u, state: %X",
            timestr,
            txCount,
            (float) txBytes / (1 << 10),
            rxBytes,
            login,
            feedid,
            state);
    }
    uint32_t txCount = 0;
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t lastSyncTime = 0;
    uint16_t feedid = 0;
    uint32_t startTime = 0;
    bool login = false;
};

class TeleClientUDP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect();
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void inbound();
    bool verifyChecksum(char* data);
    void shutdown();
#if NET_DEVICE == NET_WIFI
    UDPClientWIFI net;
#elif NET_DEVICE == NET_SIM800
    UDPClientSIM800 net;
#elif NET_DEVICE == NET_SIM5360
    UDPClientSIM5360 net;
#elif NET_DEVICE == NET_SIM7600
    UDPClientSIM7600 net;
#elif NET_DEVICE == NET_WIFI_MESH
    ClientWiFiMesh net;
#else
    ClientSerial net;
#endif
};

class TeleClientHTTP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect();
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void shutdown();
#if NET_DEVICE == NET_WIFI
    HTTPClientWIFI net;
#elif NET_DEVICE == NET_SIM800
    HTTPClientSIM800 net;
#elif NET_DEVICE == NET_SIM5360
    HTTPClientSIM5360 net;
#elif NET_DEVICE == NET_SIM7600
    HTTPClientSIM7600 net;
#endif
private:
    bool started = false;
};