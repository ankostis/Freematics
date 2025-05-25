#pragma once

#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

class CStorage;

class CStorage {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
<<<<<<< HEAD:firmware_v5/telelogger/telelogger.h
    void log(uint16_t pid, uint8_t values[], uint8_t count)
    {
        char buf[256];
        byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
        for (byte m = 1; m < count; m++) {
            n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
        }
        dispatch(buf, n);
    }
    void log(uint16_t pid, uint16_t values[], uint8_t count)
    {
        char buf[256];
        byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
        for (byte m = 1; m < count; m++) {
            n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
        }
        dispatch(buf, n);
    }
    void log(uint16_t pid, uint32_t values[], uint8_t count)
    {
        char buf[256];
        byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, values[0]);
        for (byte m = 1; m < count; m++) {
            n += snprintf(buf + n, sizeof(buf) - n, ";%u", values[m]);
        }
        dispatch(buf, n);
    }
    void log(uint16_t pid, int32_t values[], uint8_t count)
    {
        char buf[256];
        byte n = snprintf(buf, sizeof(buf), "%X%c%d", pid, m_delimiter, values[0]);
        for (byte m = 1; m < count; m++) {
            n += snprintf(buf + n, sizeof(buf) - n, ";%d", values[m]);
        }
        dispatch(buf, n);
    }
    void log(uint16_t pid, float values[], uint8_t count)
    {
        char buf[256];
        byte n = snprintf(buf, sizeof(buf), "%X%c", pid, m_delimiter);
        for (byte m = 0; m < count && n < sizeof(buf) - 3; m++) {
            if (m > 0) buf[n++] = ';';
            if (values[m] > -0.005 && values[m] < 0.005) {
                buf[n++] = '0';
                buf[n] = 0;
            } else {
                n += snprintf(buf + n, sizeof(buf) - n, "%f", values[m]);
            }
        }
        dispatch(buf, n);
    }
    void timestamp(uint32_t ts)
    {
        log(PID_TIMESTAMP, &ts, 1);
    }
    virtual void purge() { m_samples = 0; }
    uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len)
    {
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write(' ');
        m_samples++;
    }
=======
    virtual void log(uint16_t pid, uint8_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint16_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint32_t values[], uint8_t count);
    virtual void log(uint16_t pid, int32_t values[], uint8_t count);
    virtual void log(uint16_t pid, float values[], uint8_t count, const char* fmt = "%f");
    virtual void timestamp(uint32_t ts);
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len);
>>>>>>> origin/master:firmware_v5/telelogger/telestore.h
protected:
    byte checksum(const char* data, int len);
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    int m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
public:
    void init(char* cache, unsigned int cacheSize)
    {
        m_cacheSize = cacheSize;
        m_cache = cache;
    }
    void uninit()
    {
        if (m_cache) {
            delete m_cache;
            m_cache = 0;
            m_cacheSize = 0;
        }
    }
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
<<<<<<< HEAD:firmware_v5/telelogger/telelogger.h
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = m_cacheSize - m_cacheBytes - len - 3;
        if (remain < 0) {
          // m_cache full
          return;
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
        m_samples++;
    }

    void header(const char* devid)
    {
        m_cacheBytes = sprintf(m_cache, "%s#", devid);
    }
    void tailer()
    {
        if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
        m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%02X", (unsigned int)checksum(m_cache, m_cacheBytes));
    }
    void untailer()
    {
        char *p = strrchr(m_cache, '*');
        if (p) {
            *p = ',';
            m_cacheBytes = p + 1 - m_cache;
        }
    }
=======
    void dispatch(const char* buf, byte len);
    void header(const char* devid);
    void tailer();
    void untailer();
>>>>>>> origin/master:firmware_v5/telelogger/telestore.h
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

class FileLogger : public CStorage {
public:
    FileLogger() { m_delimiter = ','; }
<<<<<<< HEAD:firmware_v5/telelogger/telelogger.h
    virtual void dispatch(const char* buf, byte len)
    {
        if (m_id == 0) return;

        if (m_data_file.write((uint8_t*)buf, len) != len) {
            // try again
            if (m_data_file.write((uint8_t*)buf, len) != len) {
                ESP_LOGE(TAG, "Failed twice writing data to file: %s.", m_data_file.path());
                end();
                return;
            }
        }
        m_data_file.write('\n');
        m_size += (len + 1);
    }
    virtual uint32_t size()
    {
        return m_size;
    }
    void end()
=======
    virtual void dispatch(const char* buf, byte len);
    virtual uint32_t size() { return m_size; }
    virtual void end()
>>>>>>> origin/master:firmware_v5/telelogger/telestore.h
    {
        m_data_file.close();
        m_id = 0;
        m_size = 0;
    }
    virtual void flush()
    {
        m_data_file.flush();
    }
protected:
    int getFileID(File& root);
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_size = 0;
    uint32_t m_id = 0;
    File m_data_file;
};

/** ATTENTION: must have enabled `SD` instance before calling `begin()`. */
class SDLogger : public FileLogger {
public:
<<<<<<< HEAD:firmware_v5/telelogger/telelogger.h
    uint32_t begin()
    {
        File root = SD.open("/DATA");
        m_id = getFileID(root);
        if (m_id == 0) {
            SD.mkdir("/DATA");
            m_id = 1;
        }
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        ESP_LOGI(TAG, "Opening SD file: %s", path);
        m_data_file = SD.open(path, FILE_WRITE);
        if (!m_data_file) {
            ESP_LOGE(TAG, "Failed opening SD file: %s", path);
            m_id = 0;
        }
        m_dataCount = 0;
        return m_id;
    }
    void flush()
    {
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        m_data_file.close();
        m_data_file = SD.open(path, FILE_APPEND);
        if (!m_data_file) {
            ESP_LOGE(TAG, "Failed flushing SD file: %s", path);
        }
    }
=======
    bool init();
    uint32_t begin();
    void flush();
>>>>>>> origin/master:firmware_v5/telelogger/telestore.h
};

/** ATTENTION: must have enabled `SPIFFS` instance before calling `begin()`. */
class SPIFFSLogger : public FileLogger {
public:
<<<<<<< HEAD:firmware_v5/telelogger/telelogger.h

    uint32_t begin()
    {
        File root = SPIFFS.open("/");
        m_id = getFileID(root);
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        ESP_LOGI(TAG, "Opening SPIFFS file: %s", path);
        m_data_file = SPIFFS.open(path, FILE_WRITE);
        if (!m_data_file) {
            ESP_LOGE(TAG, "Failed opening SPIFFS file: %s", path);
            m_id = 0;
        }
        m_dataCount = 0;
        return m_id;
    }
private:
    void purge()
    {
        // remove oldest file when unused space is insufficient
        File root = SPIFFS.open("/");
        File file;
        int idx = 0;
        while(file = root.openNextFile()) {
            if (!strncmp(file.name(), "/DATA/", 6)) {
                unsigned int n = atoi(file.name() + 6);
                if (n != 0 && (idx == 0 || n < idx)) idx = n;
            }
        }
        if (idx) {
            m_data_file.close();
            char path[32];
            sprintf(path, "/DATA/%u.CSV", idx);
            SPIFFS.remove(path);
            ESP_LOGI(TAG, "Purged SPIFFS file: %s", path);
            sprintf(path, "/DATA/%u.CSV", m_id);
            m_data_file = SPIFFS.open(path, FILE_APPEND);
            if (!m_data_file) m_id = 0;
        }
    }
=======
    bool init();
    uint32_t begin();
private:
    void purge();
>>>>>>> origin/master:firmware_v5/telelogger/telestore.h
};
