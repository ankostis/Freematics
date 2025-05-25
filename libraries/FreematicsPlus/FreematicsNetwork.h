/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#ifndef FREEMATICS_NETWORK
#define FREEMATICS_NETWORK

# include <map>
# include <string>
# include <sstream>

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "FreematicsBase.h"


// ESP_IDF logging tags used
inline constexpr const char TAG_WIFI[] = "WIFI";
inline constexpr const char TAG_CELL[] = "CELL";
inline constexpr const char TAG_CELLHTTP[] = "CELLHTTP";
inline constexpr const char TAG_CELLUDP[] = "CELLUDP";
inline constexpr const char TAG_SIM7600[] = "SIM7600";
inline constexpr const char TAG_SIM7070[] = "SIM7070";
inline constexpr const char TAG_HTTP[] = "NetHTTP";

#define XBEE_BAUDRATE 115200
#define HTTP_CONN_TIMEOUT 5000

#define RECV_BUF_SIZE 512

typedef enum {
  METHOD_GET = 0,
  METHOD_POST,
} HTTP_METHOD;

typedef enum {
    HTTP_DISCONNECTED = 0,
    HTTP_CONNECTED,
    HTTP_SENT,
    HTTP_ERROR,
} HTTP_STATES;

typedef struct {
    float lat;
    float lng;
    uint8_t year; /* year past 2000, e.g. 15 for 2015 */
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} NET_LOCATION;

class HTTPClient
{
public:
    HTTP_STATES state() { return m_state; }
    uint16_t code() { return m_code; }
protected:
    std::string genHeader(HTTP_METHOD method, const char* path, const char* payload, int payloadSize);
    HTTP_STATES m_state = HTTP_DISCONNECTED;
    uint16_t m_code = 0;
    std::string m_host;
};

class ClientWIFI
{
public:
    bool begin(std::map<std::string, std::string> ssids);
    bool reconnect();
    void end();
    bool setup(unsigned int timeout = 5000);
    std::string getIP();
    int getSignal() { return 0; }
    const char* deviceName() { return "WiFi"; }
    /**
     * Dump (SSIDs, RSSI) in the log all WiFis in the area.
     *
     * return: the number of SSIDs scanned
     *
     * SIDE_EFFECT:
     *   Switches to Station-mode and disconnects from SSID,
     *   so better call `WiFi.reconnect()` afterwards.
     */
    int listAPs();
    std::string ssid() { return std::string(WiFi.SSID().c_str()); }
    std::string psk() { return std::string(WiFi.psk().c_str()); }
    bool connected() { return WiFi.isConnected(); }
    int RSSI() { return WiFi.RSSI(); }
protected:
};

class WifiUDP : public ClientWIFI
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    int receive(char* buffer, int bufsize, unsigned int timeout = 100);
    virtual std::string queryIP(const char* host);
private:
    IPAddress udpIP;
    uint16_t udpPort;
    WiFiUDP udp;
};

class WifiHTTP : public HTTPClient, public ClientWIFI
{
public:
    bool open(const char* host = 0, uint16_t port = 0);
    void close();
    bool send(HTTP_METHOD method, const char* path, const char* payload = 0, int payloadSize = 0);
    char* receive(char* buffer, int bufsize, int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
private:
    WiFiClient client;
};

typedef enum {
    CELL_SIM7600 = 0,
    CELL_SIM7670 = 1,
    CELL_SIM7070 = 2,
    CELL_SIM5360 = 3
} CELL_TYPE;

class CellSIMCOM
{
public:
    virtual bool begin(CFreematics* device);
    virtual void end();
    virtual bool setup(const char* apn, const char* username = 0, const char* password = 0, unsigned int timeout = 30000);
    virtual bool setGPS(bool on);
    virtual std::string getIP();
    int RSSI();
    std::string getOperatorName();
    bool checkSIM(const char* pin = 0);
    virtual std::string queryIP(const char* host);
    virtual bool getLocation(GPS_DATA** pgd);
    bool check(unsigned int timeout = 0);
    char* getBuffer() { return m_buffer; }
    const char* deviceName() { return m_model; }
    char IMEI[16] = {0};
protected:
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = 0);
    virtual void inbound();
    virtual void checkGPS();
    float parseDegree(const char* s);
    char* m_buffer = 0;
    char m_model[12] = {0};
    CFreematics* m_device = 0;
    GPS_DATA* m_gps = 0;
    CELL_TYPE m_type = CELL_SIM7600;
    int m_incoming = 0;
};

class CellUDP : public CellSIMCOM
{
public:
    bool open(const char* host, uint16_t port);
    bool close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
protected:
    std::string udpIP;
    uint16_t udpPort = 0;
};

class CellHTTP : public HTTPClient, public CellSIMCOM
{
public:
    void init();
    bool open(const char* host = 0, uint16_t port = 0);
    bool close();
    bool send(HTTP_METHOD method, const char* host, uint16_t port, const char* path, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
};

#endif  // FREEMATICS_NETWORK
