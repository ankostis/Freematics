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

#define ELEMENT_UINT8 0
#define ELEMENT_UINT16 1
#define ELEMENT_UINT32 2
#define ELEMENT_INT32 3
#define ELEMENT_FLOAT 4
#define ELEMENT_FLOAT_D1 5 /* floating-point data with 1 decimal place*/
#define ELEMENT_FLOAT_D2 6 /* floating-point data with 2 decimal places*/

typedef struct {
    uint16_t pid;
    uint8_t type;
    uint8_t count;
} ELEMENT_HEAD;

// ESP_IDF logging tag used
inline constexpr const char TAG_BUF[] = "APPBUF";
inline constexpr const char TAG_UDP[] = "APPUDP";
inline constexpr const char TAG_AWIFI[] = "APPWIFI";
inline constexpr const char TAG_ACELL[] = "APPCELL";

class CBuffer
{
public:
    CBuffer(uint8_t* mem);
    void add(uint16_t pid, uint8_t type, void* values, int bytes, uint8_t count = 1);
    void add(uint16_t pid, int32_t value);
    void add(uint16_t pid, float value);
    void purge();
    void serialize(CStorage& store);
    uint32_t timestamp;
    uint16_t offset;
    uint8_t total;
    uint8_t state;
private:
    uint8_t* m_data;
};

class CBufferManager
{
public:
    void init();
    void purge();
    void free(CBuffer* slot);
    CBuffer* getFree();
    CBuffer* getOldest();
    CBuffer* getNewest();
    void showCacheStats(uint16_t state);
private:
    CBuffer** slots = 0;
    CBuffer* last = 0;
    uint32_t total = 0;
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
        startTime = millis();
    }
    virtual bool notify(byte event, const char* payload = 0) { return true; }
    virtual bool connect() { return true; }
    virtual bool transmit(const char* packetBuffer, unsigned int packetSize)  { return true; }
    void showNetStats(char *timestr, uint16_t state) {
        uint32_t t = millis() - startTime;
        sprintf(timestr,
                "%02u:%02u.%c",
                t / 60000,
                (t % 60000) / 1000,
                (t % 1000) / 100 + '0');
        ESP_LOGI(TAG_BUF,
            "Netstats: %s: packet #%i, Tx: %.2fKiB, Rx: %ib, %.2fKb/h"
            ", login: %i, feedid: %u, state: %X",
            timestr,
            txCount,
            (float) txBytes / (1 << 10),
            rxBytes,
            (txBytes + rxBytes) * 3600.0 / (millis() - startTime),
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
    uint8_t packets = 0;
    bool login = false;
};

class TeleClientUDP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect(bool quick = false);
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    /**
     * :return: true if event received OR not timeout yet
     */
    void inbound();
    bool verifyChecksum(char* data);
    void shutdown();
#if ENABLE_WIFI
    WifiUDP wifi;
#endif
    CellUDP cell;
};

class TeleClientHTTP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect(bool quick = false);
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void shutdown();
#if ENABLE_WIFI
    WifiHTTP wifi;
#endif
    CellHTTP cell;
};
