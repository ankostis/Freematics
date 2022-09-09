/*************************************************************************
* Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "FreematicsPlus.h"
#include "FreematicsGPS.h"

// TODO: merge device-temp from upstream(202204)

static TinyGPS gps;
static bool gpsHasDecodedData = false;
static uart_port_t gpsUARTNum = GPS_UART_NUM;
static Task taskGPS;
static GPS_DATA* gpsData = 0;

static uint32_t inline getCycleCount()
{
  uint32_t ccount;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

static uint8_t inline readRxPin()
{
#if PIN_GPS_UART_RXD < 32
  return (uint8_t)(GPIO.in >> PIN_GPS_UART_RXD) << 7;
#else
  return (uint8_t)(GPIO.in1.val >> (PIN_GPS_UART_RXD - 32)) << 7;
#endif
}

static void gps_decode_task(void* inst)
{
    for (;;) {
        uint8_t c = 0;
        int len = uart_read_bytes(gpsUARTNum, &c, 1, 60000 / portTICK_RATE_MS);
        if (len != 1) continue;
        //Serial.print((char)c);
        if (gps.encode(c)) {
            gpsHasDecodedData = true;
        }
    }
}

static void inline setTxPinHigh()
{
#if PIN_GPS_UART_TXD < 32
  GPIO.out_w1ts = ((uint32_t)1 << PIN_GPS_UART_TXD);
#else
  GPIO.out1_w1ts.val = ((uint32_t)1 << (PIN_GPS_UART_TXD - 32));
#endif
}

static void inline setTxPinLow()
{
#if PIN_GPS_UART_TXD < 32
  GPIO.out_w1tc = ((uint32_t)1 << PIN_GPS_UART_TXD);
#else
  GPIO.out1_w1tc.val = ((uint32_t)1 << (PIN_GPS_UART_TXD - 32));
#endif
}

static void softSerialTx(uint32_t baudrate, uint8_t c)
{
  uint32_t start = getCycleCount();
  // start bit
  setTxPinLow();
  for (uint32_t i = 1; i <= 8; i++, c >>= 1) {
    while (getCycleCount() - start < i * F_CPU / baudrate);
    if (c & 0x1)
      setTxPinHigh();
    else
      setTxPinLow();
  }
  while (getCycleCount() - start < (uint32_t)9 * F_CPU / baudrate);
  setTxPinHigh();
  while (getCycleCount() - start < (uint32_t)10 * F_CPU / baudrate);
}

static void gps_soft_decode_task(void* inst)
{
    // start receiving and decoding
    for (;;) {
        uint8_t c = 0;
        do {
            taskYIELD();
        } while (readRxPin());
        uint32_t start = getCycleCount();
        for (uint32_t i = 1; i <= 7; i++) {
            taskYIELD();
            while (getCycleCount() - start < i * F_CPU / GPS_SOFT_BAUDRATE + F_CPU / GPS_SOFT_BAUDRATE / 3);
            c = (c | readRxPin()) >> 1;
        }
        if (gps.encode(c)) {
            gpsHasDecodedData = true;
        }
        do {
            taskYIELD();
        } while (getCycleCount() - start < (uint32_t)9 * F_CPU / GPS_SOFT_BAUDRATE + F_CPU / GPS_SOFT_BAUDRATE / 2);
    }
}

extern "C" {
uint8_t temprature_sens_read();
int32_t hall_sens_read();
}

// get chip temperature sensor
int readChipTemperature()
{
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
    SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10, SENS_TSENS_CLK_DIV_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    ets_delay_us(100);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    ets_delay_us(5);
    int res = GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG, SENS_TSENS_OUT, SENS_TSENS_OUT_S);
    return (res - 32) * 5 / 9;
}

int readChipHallSensor()
{
  return hall_sens_read();
}

bool Task::create(void (*task)(void*), const char* name, int priority, int stacksize)
{
    if (xHandle) return false;
    /* Create the task, storing the handle. */
    BaseType_t xReturned = xTaskCreate(task, name, stacksize, (void*)this, priority, &xHandle);
    return xReturned == pdPASS;
}

void Task::destroy()
{
    if (xHandle) {
        void* x = xHandle;
        xHandle = 0;
        vTaskDelete((TaskHandle_t)x);
    }
}

void Task::sleep(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

bool Task::running()
{
    return xHandle != 0;
}

void Task::suspend()
{
    if (xHandle) vTaskSuspend(xHandle);
}

void Task::resume()
{
    if (xHandle) vTaskResume(xHandle);
}

Mutex::Mutex()
{
  xSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(xSemaphore);
}

void Mutex::lock()
{
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
}

void Mutex::unlock()
{
  xSemaphoreGive(xSemaphore);
}

bool CLink_UART::begin(unsigned int baudrate, int rxPin, int txPin)
{
    ESP_LOGD(TAG_LINK, "<BEGIN> UART-%i, %i, %i, %i", LINK_UART_NUM, baudrate, rxPin, txPin);
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(LINK_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(LINK_UART_NUM, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    if (uart_driver_install(LINK_UART_NUM, LINK_UART_BUF_SIZE, 0, 0, NULL, 0) != ESP_OK)
		return false;
	return true;
}

void CLink_UART::end()
{
    ESP_LOGD(TAG_LINK, "<END> UART-%i", LINK_UART_NUM);
	uart_driver_delete(LINK_UART_NUM);
}

int CLink_UART::receive(char* buffer, int bufsize, unsigned int timeout)
{
	unsigned char n = 0;
	unsigned long startTime = millis();
	unsigned long elapsed;
	for (;;) {
		elapsed = millis() - startTime;
		if (elapsed > timeout) break;
		if (n >= bufsize - 1) break;
		int len = uart_read_bytes(LINK_UART_NUM, (uint8_t*)buffer + n, bufsize - n - 1, 1);
		if (len < 0) break;
		if (len == 0) continue;
		buffer[n + len] = 0;
		if (strstr(buffer + n, "\r>")) {
			n += len;
			break;
		}
		n += len;
		if (strstr(buffer, "...")) {
			buffer[0] = 0;
			n = 0;
			timeout += OBD_TIMEOUT_LONG;
		}
	}
	ESP_LOGV(TAG_LINK, "<UART RECV> x%i |%s|", n, buffer);
	return n;
}

bool CLink_UART::send(const char* str)
{
    int len = strlen(str);
	ESP_LOGV(TAG_LINK, "<UART SEND> x%i |%s|", len, str);
	return uart_write_bytes(LINK_UART_NUM, str, len) == len;
}

int CLink_UART::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	send(cmd);
	return receive(buf, bufsize, timeout);
}

int CLink_UART::read()
{
    uint8_t c;
    if (uart_read_bytes(LINK_UART_NUM, &c, 1, 1) == 1)
        return c;
    else
        return -1;
}

bool CLink_UART::changeBaudRate(unsigned int baudrate)
{
	char buf[32];
	sprintf(buf, "ATBR1 %X\r", baudrate);
	sendCommand(buf, buf, sizeof(buf), 1000);
	delay(50);
	end();
	return begin(baudrate);
}

bool CLink_SPI::begin(unsigned int freq, int rxPin, int txPin)
{
    ESP_LOGD(TAG_SPI, "<SPI BEGIN> %i, %i, %i", freq, rxPin, txPin);
	pinMode(PIN_LINK_SPI_READY, INPUT);
	pinMode(PIN_LINK_SPI_CS, OUTPUT);
	digitalWrite(PIN_LINK_SPI_CS, HIGH);
    delay(50);
    for (uint32_t t = millis(); millis() - t < 50; ) {
        if (digitalRead(PIN_LINK_SPI_READY) == LOW) return false;
    }
	SPI.begin();
	SPI.setFrequency(freq);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	return true;
}

void CLink_SPI::end()
{
    ESP_LOGD(TAG_SPI, "<SPI END>");
	SPI.end();
}

int CLink_SPI::receive(char* buffer, int bufsize, unsigned int timeout)
{
	int n = 0;
	bool eos = false;
	bool matched = false;
	portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
	uint32_t t = millis();
	do {
		while (digitalRead(PIN_LINK_SPI_READY) == HIGH) {
			if (millis() - t > 3000) return -1;
            delay(1);
		}
    	ESP_LOGV(TAG_SPI, "<SPI RECV>");
		portENTER_CRITICAL(&m);
		digitalWrite(PIN_LINK_SPI_CS, LOW);
		while (digitalRead(PIN_LINK_SPI_READY) == LOW && millis() - t < timeout) {
			char c = SPI.transfer(' ');
			if (c == 0 && c == 0xff) continue;
			if (!eos) eos = (c == 0x9);
			if (eos) continue;
			if (!matched) {
				// match header
				if (n == 0 && c != header[0]) continue;
				if (n == bufsize - 1) continue;
				buffer[n++] = c;
				if (n == sizeof(header)) {
					matched = memcmp(buffer, header, sizeof(header)) == 0;
					if (matched) {
						n = 0;
					} else {
						memmove(buffer, buffer + 1, --n);
					}
				}
				continue;
			}
			if (n > 3 && c == '.' && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
				// SEARCHING...
				n = 0;
				timeout += OBD_TIMEOUT_LONG;
			} else {
				if (n == bufsize - 1) {
					int bytesDumped = dumpLine(buffer, n);
					n -= bytesDumped;
					ESP_LOGW(TAG_LINK, "<SPI RECV> BUFFER FULL");
				}
				buffer[n++] = c;
			}
		}
		digitalWrite(PIN_LINK_SPI_CS, HIGH);
		portEXIT_CRITICAL(&m);
	} while (!eos && millis() - t < timeout);
	if (!eos) {
		// timed out
		ESP_LOGW(TAG_SPI, "<SPI RECV> TIMEOUT");
	}
	buffer[n] = 0;
	ESP_LOGD(TAG_LINK, "<SPI RECV> x%i |%s|", n, buffer);
	// wait for READY pin to restore high level so SPI bus is released
    if (eos) while (digitalRead(PIN_LINK_SPI_READY) == LOW) delay(1);
	return n;
}

bool CLink_SPI::send(const char* str)
{
    if (digitalRead(PIN_LINK_SPI_READY) == LOW) {
    	ESP_LOGW(TAG_SPI, "<SPI SEND> NOT READY");
        return false;
    }
	portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
	int len = strlen(str);
	ESP_LOGD(TAG_SPI, "<SPI SEND> x%i |%s|", len, str);
	uint8_t tail = 0x1B;
	portENTER_CRITICAL(&m);
	digitalWrite(PIN_LINK_SPI_CS, LOW);
	delay(1);
	SPI.writeBytes((uint8_t*)header, sizeof(header));
	SPI.writeBytes((uint8_t*)str, len);
	SPI.writeBytes((uint8_t*)&tail, 1);
	delay(1);
	digitalWrite(PIN_LINK_SPI_CS, HIGH);
	portEXIT_CRITICAL(&m);
    return true;
}

int CLink_SPI::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	uint32_t t = millis();
	int n = 0;
	for (byte i = 0; i < 30 && millis() - t < timeout; i++) {
		if (!send(cmd)) {
            delay(50);
            continue;
        }
		n = receive(buf, bufsize, timeout);
		if (n == -1) {
			ESP_LOGV(TAG_SPI, "_");
			n = 0;
			continue;
		}
		if (n == 0 || (buf[1] != 'O' && !memcmp(buf + 5, "NO DATA", 7))) {
			// data not ready
			delay(50);
		} else {
	  		break;
		}
	}
	return n;
}

void FreematicsESP32::gpsEnd()
{
    // uninitialize
    ESP_LOGI(TAG_GNSS, "<END>");
    if ((m_flags & FLAG_GNSS_USE_LINK)) {
        char buf[16];
        link->sendCommand("ATGPSOFF\r", buf, sizeof(buf), 0);
    } else {
        taskGPS.destroy();
        if (!(m_flags & FLAG_GNSS_SOFT_SERIAL)) {
            ESP_LOGD(TAG_GNSS, "<END> UART-%i", gpsUARTNum);
            uart_driver_delete(gpsUARTNum);
        }
        digitalWrite(m_pinGPSPower, LOW);
    }
}

bool FreematicsESP32::gpsBegin(int baudrate)
{
    ESP_LOGV(TAG_GNSS, "<BEGIN>");
    // TODO: merge upstream(202204) `gpsBegin()` for set-baud/Link-GNSS/C3 enhancements.

    // try co-processor GPS link
    if (m_flags & FLAG_GNSS_USE_LINK) {
        char buf[128];
        link->sendCommand("ATGPSON\r", buf, sizeof(buf), 100);
        m_flags |= FLAG_GNSS_USE_LINK;
        uint32_t t = millis();
        bool success = false;
        do {
            if (gpsGetNMEA(buf, sizeof(buf)) > 0 && strstr(buf, ("$G"))) {
                success = true;
                break;
            }
        } while (millis() - t < 1000);
        if (success) {
            gpsData = new GPS_DATA;
            memset(gpsData, 0, sizeof(GPS_DATA));
            m_pinGPSPower = 0;

            ESP_LOGI(TAG_GNSS, "<BEGIN> through LINK");
            return true;
        }
        link->sendCommand("ATGPSOFF\r", buf, sizeof(buf), 100);
        m_flags &= ~FLAG_GNSS_USE_LINK;
    }
    // switch on GNSS power
    if (m_pinGPSPower) pinMode(m_pinGPSPower, OUTPUT);
    if (!(m_flags & FLAG_GNSS_SOFT_SERIAL)) {
        uart_config_t uart_config = {
            .baud_rate = baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
        };
        bool legacy = devType <= 13;
        ESP_LOGD(
                TAG_GNSS,
                "<BEGIN?> UART-%i(legacy: %i, txpin: %i, rxpin: %i, baud: %i)",
                gpsUARTNum,
                legacy,
                legacy ? PIN_GPS_UART_TXD2 : PIN_GPS_UART_TXD,
                legacy ? PIN_GPS_UART_RXD2 : PIN_GPS_UART_RXD,
                baudrate);
        // configure UART parameters
        uart_param_config(gpsUARTNum, &uart_config);
        // set UART pins
        uart_set_pin(gpsUARTNum, legacy ? PIN_GPS_UART_TXD2 : PIN_GPS_UART_TXD, legacy ? PIN_GPS_UART_RXD2 : PIN_GPS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        // install UART driver
        uart_driver_install(gpsUARTNum, UART_BUF_SIZE, 0, 0, NULL, 0);
        // turn on GPS power
        if (m_pinGPSPower) digitalWrite(m_pinGPSPower, HIGH);
        delay(100);
        // start decoding task
        taskGPS.create(gps_decode_task, "GPS", 1);
    } else {
        ESP_LOGD(
            TAG_GNSS, "<BEGIN?> UART-soft(%i, %i)",
            PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
        pinMode(PIN_GPS_UART_RXD, INPUT);
        pinMode(PIN_GPS_UART_TXD, OUTPUT);
        setTxPinHigh();

        // turn on GPS power
        delay(20);
        if (m_pinGPSPower) digitalWrite(m_pinGPSPower, HIGH);
        delay(100);

        // start GPS decoding task (soft serial)
        taskGPS.create(gps_soft_decode_task, "GPS", 1);
    }

    // test run for a while to see if there is data decoded
    uint16_t s1 = 0, s2 = 0;
    gps.stats(&s1, 0);
    for (int i = 0; i < 10; i++) {
        if (m_flags & FLAG_GNSS_SOFT_SERIAL) {
            // switch M8030 GNSS to 38400bps
            const uint8_t packet1[] = {0x0, 0x0, 0xB5, 0x62, 0x06, 0x0, 0x14, 0x0, 0x01, 0x0, 0x0, 0x0, 0xD0, 0x08, 0x0, 0x0, 0x0, 0x96, 0x0, 0x0, 0x7, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x93, 0x90};
            const uint8_t packet2[] = {0xB5, 0x62, 0x06, 0x0, 0x1, 0x0, 0x1, 0x8, 0x22};
            for (int i = 0; i < sizeof(packet1); i++) softSerialTx(baudrate, packet1[i]);
            delay(20);
            for (int i = 0; i < sizeof(packet2); i++) softSerialTx(baudrate, packet2[i]);
        }
        delay(200);
        gps.stats(&s2, 0);
        if (s1 != s2) {
            // data is coming in
            if (!gpsData) gpsData = new GPS_DATA;
            memset(gpsData, 0, sizeof(GPS_DATA));

            ESP_LOGI(
                TAG_GNSS, "<BEGIN> using UART-%i(99=soft)",
                (m_flags & FLAG_GNSS_SOFT_SERIAL)? 99 : gpsUARTNum);
            return true;
        }
    }
    // when no data coming in
    gpsEnd();
    return false;
}

bool FreematicsESP32::gpsGetData(GPS_DATA** pgd)
{
    bool ret;
    ESP_LOGV(TAG_GNSS, "<READ ASKED>");
    if (!gpsData) return false;
    if (pgd) *pgd = gpsData;
    if (m_flags & FLAG_GNSS_USE_LINK) {
        ret = _gpsGetData_linkUart(pgd);
    } else {
        ret = _gpsGetData_tinyGps(pgd);
    }
    ESP_LOGD(
            TAG_GNSS, "[READ %s] %s: sat: %u, err: %u, hdop: %u",
            m_flags & FLAG_GNSS_USE_LINK? "LINK": "TinyGPS",
            ret? "OK": "fail",
            gpsData->sat,
            gpsData->errors,
            gpsData->hdop);
    return ret;
}

bool FreematicsESP32::_gpsGetData_linkUart(GPS_DATA** pgd)
{
    char buf[160];
    if (!link || link->sendCommand("ATGPS\r", buf, sizeof(buf), 100) == 0) {
        return false;
    }
    char *s = strstr(buf, "$GNIFO,");
    if (!s) return false;
    s += 7;
    float lat = 0;
    float lng = 0;
    float alt = 0;
    bool good = false;
    do {
        uint32_t date = atoi(s);
        if (!(s = strchr(s, ','))) break;
        uint32_t time = atoi(++s);
        if (!(s = strchr(s, ','))) break;
        if (!date) break;
        gpsData->date = date;
        gpsData->time = time;
        lat = (float)atoi(++s) / 1000000;
        if (!(s = strchr(s, ','))) break;
        lng = (float)atoi(++s) / 1000000;
        if (!(s = strchr(s, ','))) break;
        alt = (float)atoi(++s) / 100;
        good = true;
        if (!(s = strchr(s, ','))) break;
        gpsData->speed = (float)atoi(++s) / 100;
        if (!(s = strchr(s, ','))) break;
        gpsData->heading = atoi(++s) / 100;
        if (!(s = strchr(s, ','))) break;
        gpsData->sat = atoi(++s);
        if (!(s = strchr(s, ','))) break;
        gpsData->hdop = atoi(++s);
    } while(0);
    if (good && (gpsData->lat || gpsData->lng)) {
        // filter out invalid coordinates
        good = (abs(lat * 1000000 - gpsData->lat * 1000000) < 100000 && abs(lng * 1000000 - gpsData->lng * 1000000) < 100000);
    }
    if (!good) return false;
    gpsData->lat = lat;
    gpsData->lng = lng;
    gpsData->alt = alt;
    gpsData->ts = millis();
    return true;
}

bool FreematicsESP32::_gpsGetData_tinyGps(GPS_DATA** pgd)
{
    gps.stats(&gpsData->sentences, &gpsData->errors);
    if (!gpsHasDecodedData) return false;
    long lat, lng;
    bool good = true;
    gps.get_position(&lat, &lng, 0);
    if (gpsData->lat || gpsData->lng) {
        // filter out invalid coordinates
        good = (abs(lat - gpsData->lat * 1000000) < 100000 && abs(lng - gpsData->lng * 1000000) < 100000);
    }
    if (!good) return false;
    gpsData->ts = millis();
    gpsData->lat = (float)lat / 1000000;
    gpsData->lng = (float)lng / 1000000;
    gps.get_datetime((unsigned long*)&gpsData->date, (unsigned long*)&gpsData->time, 0);
    long alt = gps.altitude();
    if (alt != TinyGPS::GPS_INVALID_ALTITUDE) gpsData->alt = (float)alt / 100;
    unsigned long knot = gps.speed();
    if (knot != TinyGPS::GPS_INVALID_SPEED) gpsData->speed = (float)knot / 100;
    unsigned long course = gps.course();
    if (course < 36000) gpsData->heading = course / 100;
    unsigned short sat = gps.satellites();
    if (sat != TinyGPS::GPS_INVALID_SATELLITES) gpsData->sat = sat;
    unsigned long hdop = gps.hdop();
    gpsData->hdop = hdop > 2550 ? 255 : hdop / 10;
    gpsHasDecodedData = false;
    return true;
}

int FreematicsESP32::gpsGetNMEA(char* buffer, int bufsize)
{
    if (m_flags & FLAG_GNSS_USE_LINK) {
        return link->sendCommand("ATGRR\r", buffer, bufsize, 200);
    } else {
        return 0;
    }
}

void FreematicsESP32::gpsSendCommand(const char* string, int len)
{
#if !GPS_SOFT_SERIAL
    if (taskGPS.running())
        uart_write_bytes(gpsUARTNum, string, len);
#endif
}

bool FreematicsESP32::xbBegin(unsigned long baudrate, int pinRx, int pinTx)
{
    ESP_LOGD(TAG_GSM, "<BEGIN> UART-%i, %i, %i", BEE_UART_NUM, pinRx, pinTx);
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    //Configure UART parameters
    uart_param_config(BEE_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(BEE_UART_NUM, pinTx, pinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    uart_driver_install(BEE_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

#ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, LOW);
#endif
    return true;
}

void FreematicsESP32::xbEnd()
{
    ESP_LOGD(TAG_GSM, "<END> UART-%i", BEE_UART_NUM);
    uart_driver_delete(BEE_UART_NUM);
    digitalWrite(PIN_BEE_PWR, LOW);
}

void FreematicsESP32::xbWrite(const char* cmd)
{
    int len = strlen(cmd);
    uart_write_bytes(BEE_UART_NUM, cmd, len);
    ESP_LOGV(TAG_GSM, "<SEND> x%i |%s|", len, cmd);
}

void FreematicsESP32::xbWrite(const char* data, int len)
{
    ESP_LOGV(TAG_GSM, "<SEND> x%i |%.*s|", len, len, data);
    uart_write_bytes(BEE_UART_NUM, data, len);
}

int FreematicsESP32::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
    int recv = 0;
    uint32_t t = millis();
    do {
        uint8_t c;
        int len = uart_read_bytes(BEE_UART_NUM, &c, 1, 0);
        if (len == 1) {
            if (c >= 0xA && c <= 0x7E) {
                buffer[recv++] = c;
            }
        } else if (recv > 0) {
            break;
        }
    } while (recv < bufsize && millis() - t < timeout);

    ESP_LOGV(TAG_GSM, "<RECV> x%i |%.*s|", recv, recv, buffer);
    return recv;
}

int FreematicsESP32::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
    int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 50);
		if (n > 0) {
			buffer[bytesRecv + n] = 0;
			ESP_LOGV(TAG_GSM, "<RECV> x%i |%s|", n, buffer + bytesRecv);
			bytesRecv += n;
			buffer[bytesRecv] = 0;
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
		} else if (n == -1) {
			// an erroneous reading
			ESP_LOGW(TAG_GSM, "RECV ERROR");
			break;
		}
	} while (millis() - t < timeout);
	buffer[bytesRecv] = 0;
	return 0;
}

void FreematicsESP32::xbPurge()
{
    ESP_LOGD(TAG_LINK, "<PURGE>");
    uart_flush(BEE_UART_NUM);
}

void FreematicsESP32::xbTogglePower()
{
#ifdef PIN_BEE_PWR
    ESP_LOGD(TAG_GNSS, "Toggle GSM POWER pin %i x2 times...", PIN_BEE_PWR);
    digitalWrite(PIN_BEE_PWR, HIGH);
    delay(100);
	digitalWrite(PIN_BEE_PWR, LOW);
	delay(200);  // TODO: `delay(1010)` in upstream(202204)
    digitalWrite(PIN_BEE_PWR, HIGH);
#endif
    delay(100);
    digitalWrite(PIN_BEE_PWR, LOW);
    ESP_LOGV(TAG_GNSS, "Finished toggling GSM power");
}

byte FreematicsESP32::getDeviceType()
{
    if (!link) return 0;
    char buf[32];
    if (link->sendCommand("ATI\r", buf, sizeof(buf), 1000)) {
        char *p = strstr(buf, "OBD");
        if (p && (p = strchr(p, ' '))) {
            p += 2;
            if (isdigit(*p) && *(p + 1) == '.' && isdigit(*(p + 2))) {
                devType = (*p - '0') * 10 + (*(p + 2) - '0');
                return devType;
            }
        }
    }
	return 0;
}

bool FreematicsESP32::reactivateLink()
{
    ESP_LOGD(TAG_LINK, "<REACTIVATE?> %i", bool(link));
    if (!link) return false;
    for (int n = 0; n < 30; n++) {
        char buf[32];
        if (link->sendCommand("ATI\r", buf, sizeof(buf), 1000)) return true;
    }
    return false;
}

void FreematicsESP32::resetLink()
{
    ESP_LOGD(TAG_LINK, "<RESET>");
    char buf[16];
    if (link) link->sendCommand("ATR\r", buf, sizeof(buf), 100);
    if (devType == 11 || devType >= 14) {
        digitalWrite(PIN_LINK_RESET, LOW);
        delay(50);
        digitalWrite(PIN_LINK_RESET, HIGH);
    }
}

bool FreematicsESP32::begin(bool useCoProc, bool useCellular)
{
    ESP_LOGD(TAG, "<BEGIN ESP32> %i, %i", useCoProc, useCellular);
    if (link) return false;

    pinMode(PIN_LINK_RESET, OUTPUT);
    digitalWrite(PIN_LINK_RESET, HIGH);

    // set watchdog timeout to 600 seconds
    esp_task_wdt_init(600, 0);

    m_flags = 0;
    m_pinGPSPower = PIN_GPS_POWER;

    if (useCoProc) do {
        CLink_UART *linkUART = new CLink_UART;
        //linkUART->begin(115200);
        //char buf[16];
        // lift baudrate to 25600bps
        //linkUART->sendCommand("ATBR1 3E800\r", buf, sizeof(buf), 50);
        //linkUART->end();
        if (linkUART->begin()) {
            link = linkUART;
            for (byte n = 0; n < 3 && !getDeviceType(); n++);
            if (devType) {
                m_flags = FLAG_USE_UART_LINK;
                if (devType == 13 || devType == 14 || devType == 15) {
                    m_flags |= FLAG_GNSS_USE_LINK;
                } else if (devType == 12) {
                    m_pinGPSPower = PIN_GPS_POWER2;
                }
                break;
            }
            link = 0;
            linkUART->end();
        }
        delete linkUART;
        linkUART = 0;
#if 0
        CLink_SPI *linkSPI = new CLink_SPI;
        if (linkSPI->begin()) {
            link = linkSPI;
            for (byte n = 0; n < 10 && !getDeviceType(); n++);
            if (devType) {
                m_pinGPSPower = PIN_GPS_POWER2;
                break;
            }
            link = 0;
            linkSPI->end();
        }
        delete linkSPI;
        linkSPI = 0;
#endif
        useCoProc = false;
    } while(0);

    if (useCellular) {
        int pinRx = PIN_BEE_UART_RXD;
        int pinTx = PIN_BEE_UART_TXD;
        if (devType == 13) {
            pinRx = PIN_BEE_UART_RXD2;
            pinTx = PIN_BEE_UART_TXD2;
        } else if ((devType == 11 && !(m_flags & FLAG_USE_UART_LINK)) || devType == 12 || devType == 0) {
            pinRx = PIN_BEE_UART_RXD3;
            pinTx = PIN_BEE_UART_TXD3;
        }
        xbBegin(XBEE_BAUDRATE, pinRx, pinTx);
        m_flags |= FLAG_USE_CELL;
        if ((m_flags & FLAG_USE_UART_LINK) && (devType == 11 || devType == 12 || devType == 16)) {
            m_flags |= FLAG_GNSS_SOFT_SERIAL;
        }
    }
    gpsUARTNum = useCoProc ? GPS_UART_NUM : LINK_UART_NUM;
    return devType != 0;
}
