#include <FreematicsPlus.h>
#include "telestore.h"

void CStorage::log(uint16_t pid, uint8_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint16_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%u", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, int32_t values[], uint8_t count)
{
    char buf[256];
    byte n = snprintf(buf, sizeof(buf), "%X%c%d", pid, m_delimiter, values[0]);
    for (byte m = 1; m < count; m++) {
        n += snprintf(buf + n, sizeof(buf) - n, ";%d", values[m]);
    }
    dispatch(buf, n);
}

void CStorage::log(uint16_t pid, float values[], uint8_t count, const char* fmt)
{
    char buf[256];
    char *p = buf + snprintf(buf, sizeof(buf), "%X%c", pid, m_delimiter);
    for (byte m = 0; m < count && (p - buf) < sizeof(buf) - 3; m++) {
        if (m > 0) *(p++) = ';';
        int l = snprintf(p, sizeof(buf) - (p - buf), fmt, values[m]);
        char *q = strchr(p, '.');
        if (q && atoi(q + 1) == 0) {
            *q = 0;
            if (*p == '-' && *(p + 1) == '0') {
                *p = '0';
                *(++p) = 0;
            } else {
                p = q;
            }
        } else {
            p += l;
        }
    }
    dispatch(buf, (int)(p - buf));
}

void CStorage::timestamp(uint32_t ts)
{
    log(PID_TIMESTAMP, &ts, 1);
}

void CStorage::dispatch(const char* buf, byte len)
{
    // output data via serial
    Serial.write((uint8_t*)buf, len);
    Serial.write(' ');
    m_samples++;
}

byte CStorage::checksum(const char* data, int len)
{
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

void CStorageRAM::dispatch(const char* buf, byte len)
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

void CStorageRAM::header(const char* devid)
{
    m_cacheBytes = sprintf(m_cache, "%s#", devid);
}

void CStorageRAM::tailer()
{
    if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
    m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
}

void CStorageRAM::untailer()
{
    char *p = strrchr(m_cache, '*');
    if (p) {
        *p = ',';
        m_cacheBytes = p + 1 - m_cache;
    }
}

void FileLogger::dispatch(const char* buf, byte len)
{
    if (m_id == 0) return;

    if (m_data_file.write((uint8_t*)buf, len) != len) {
        // try again
        if (m_data_file.write((uint8_t*)buf, len) != len) {
            ESP_LOGE(TAG_FILE, "Failed twice writing data to file '%s'. End file logging.", m_data_file.path());
            end();
            return;
        }
    }
    m_data_file.write('\n');
    m_size += (len + 1);
}

int FileLogger::getFileID(File& root)
{
    if (root) {
        File file;
        int id = 0;
        while(file = root.openNextFile()) {
            char *p = strrchr(file.name(), '/');
            unsigned int n = atoi(p ? p + 1 : file.name());
            if (n > id) id = n;
        }
        return id + 1;
    } else {
        return 0;
    }
}

bool SDLogger::init()
{
    SPI.begin();
    if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ)) {
        unsigned int total = SD.totalBytes() >> 20;
        unsigned int used = SD.usedBytes() >> 20;
        ESP_LOGI(TAG_SD, "%d/%d MB total/used", total, used);
        return true;
    } else {
        ESP_LOGW(TAG_SD, "NO SD CARD");
        return false;
    }
}

uint32_t SDLogger::begin()
{
    File root = SD.open("/DATA");
    m_id = getFileID(root);
    if (m_id == 0) {
        SD.mkdir("/DATA");
        m_id = 1;
    }
    char path[24];
    sprintf(path, "/DATA/%u.CSV", m_id);
    ESP_LOGI(TAG_SD, "Opening file: %s", path);
    m_data_file = SD.open(path, FILE_WRITE);
    if (!m_data_file) {
        ESP_LOGE(TAG_SD, "Failed opening file: %s", path);
        m_id = 0;
    }
    m_dataCount = 0;
    return m_id;
}

void SDLogger::flush()
{
    char path[24];
    sprintf(path, "/DATA/%u.CSV", m_id);
    m_data_file.close();
    m_data_file = SD.open(path, FILE_APPEND);
    if (!m_data_file) {
        ESP_LOGE(TAG_SD, "Failed flushing file: %s", path);
    }
}

bool SPIFFSLogger::init()
{
    bool mounted = SPIFFS.begin();
    if (!mounted) {
        ESP_LOGI(TAG_SPIFFS, "Formatting...");
        mounted = SPIFFS.begin(true);
    }
    if (mounted) {
        ESP_LOGI(TAG_SPIFFS, "%d/%d bytes total/used", SPIFFS.totalBytes(), SPIFFS.usedBytes());
    } else {
        ESP_LOGE(TAG_SPIFFS, "No SPIFFS");
    }
    return mounted;
}

uint32_t SPIFFSLogger::begin()
{
    File root = SPIFFS.open("/");
    m_id = getFileID(root);
    char path[24];
    sprintf(path, "/DATA/%u.CSV", m_id);
    ESP_LOGI(TAG_SPIFFS, "Opening file: %s", path);
    m_data_file = SPIFFS.open(path, FILE_WRITE);
    if (!m_data_file) {
        ESP_LOGE(TAG_SPIFFS, "Failed opening file: %s", path);
        m_id = 0;
    }
    m_dataCount = 0;
    return m_id;
}

void SPIFFSLogger::purge()
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
        ESP_LOGI(TAG_SPIFFS, "Purged file: %s", path);
        sprintf(path, "/DATA/%u.CSV", m_id);
        m_data_file = SPIFFS.open(path, FILE_APPEND);
        if (!m_data_file) m_id = 0;
    }
}
