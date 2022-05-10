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
    void log(uint16_t pid, int value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%d", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    void log(uint16_t pid, uint32_t value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%u", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    void log(uint16_t pid, float value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%f", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    void log(uint16_t pid, float value[])
    {
        char buf[48];
        byte len = sprintf(buf, "%X%c%.2f;%.2f;%.2f", pid, m_delimiter, value[0], value[1], value[2]);
        dispatch(buf, len);
    }
    void timestamp(uint32_t ts)
    {
        log(0, ts);
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
protected:
    byte checksum(const char* data, int len)
    {
        byte sum = 0;
        for (int i = 0; i < len; i++) sum += data[i];
        return sum;
    }
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    uint16_t m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
public:
    bool init(unsigned int cacheSize)
    {
      if (m_cacheSize != cacheSize) {
        uninit();
        m_cache = new char[m_cacheSize = cacheSize];
      }
      return true;
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
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

class FileLogger : public CStorage {
public:
    FileLogger() { m_delimiter = ','; }
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
    int getFileID(File& root)
    {
        if (root) {
            File file;
            int id = 0;
            while(file = root.openNextFile()) {
                Serial.println(file.name());
                if (!strncmp(file.name(), "/DATA/", 6)) {
                    unsigned int n = atoi(file.name() + 6);
                    if (n > id) id = n;
                }
            }
            return id + 1;
        } else {
            return 1;
        }
    }
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_size = 0;
    uint32_t m_id = 0;
    File m_data_file;
};

class SDLogger : public FileLogger {
public:
    bool init()
    {
        SPI.begin();
        if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ)) {
            unsigned int total = SD.totalBytes() >> 20;
            unsigned int used = SD.usedBytes() >> 20;
            ESP_LOGI(TAG, "SD: %i MB total, %i MB used", total, used);
            return true;
        } else {
            ESP_LOGE(TAG, "No SD card");
            return false;
        }
    }
    uint32_t begin()
    {
        File root = SD.open("/DATA");
        m_id = getFileID(root);
        SD.mkdir("/DATA");
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
};

class SPIFFSLogger : public FileLogger {
public:
    bool init()
    {
        bool mounted = SPIFFS.begin();
        if (!mounted) {
            ESP_LOGI(TAG, "Formatting SPIFFS...");
            mounted = SPIFFS.begin(true);
        }
        if (mounted) {
            ESP_LOGI(
                TAG,
                "SPIFFS: %i bytes total, %i bytes used",
                SPIFFS.totalBytes(), SPIFFS.usedBytes());
        } else {
            ESP_LOGE(TAG, "No SPIFFS");
        }
        return mounted;
    }
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
};
