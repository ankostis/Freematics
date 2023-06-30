/*************************************************************************
* Arduino Library for Freematics ONE/ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2012-2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#ifndef FREEMATICS_OBD
#define FREEMATICS_OBD

#include <Arduino.h>

#include "FreematicsBase.h"
#include "utility/OBD.h"

// ESP_IDF logging tags used
inline constexpr const char TAG_OBD[] = "OBD";

#define OBD_TIMEOUT_SHORT 1000 /* ms */
#define OBD_TIMEOUT_LONG 10000 /* ms */

int dumpLine(char* buffer, int len);
uint16_t hex2uint16(const char *p);
byte hex2uint8(const char *p);

class COBD
{
public:
	void begin(CLink* link) { this->link = link; }
	/**
	 * initialize OBD-II connection
	 *
	 * :param obd_alt_init_cmds:
	 * 		A list of ELM327-like Protocol AT-commands to send to the OBD-coprocessor
	 * 		if initialization fails AND `protocol` is NOT `PROTO_J1939`.
	 */
	bool init(
		OBD_PROTOCOLS protocol = PROTO_AUTO,
		std::vector<std::vector<std::string>> obd_alt_init_cmds={}
	);
	// reset OBD-II connection
	void reset();
	// un-initialize OBD-II connection
	void uninit();
	// set serial baud rate
	bool setBaudRate(unsigned long baudrate);
	// get connection state
	OBD_STATES getState() { return m_state; }
	// read specified OBD-II PID value
	bool readPID(byte pid, int& result);
	// read multiple OBD-II PID values, return number of values obtained
	byte readPID(const byte pid[], byte count, int result[]);
	// read OBD-II multiple PID (don't confuse with array of PID)
	bool readPIDMulti(DS_CAN_MSG* obdDataMulti);
	// set device into low power mode
	void enterLowPowerMode();
	// wake up device from low power mode
	void leaveLowPowerMode();
	// read diagnostic trouble codes (return number of DTCs read)
	int readDTC(uint16_t codes[], byte maxCodes = 1);
	// clear diagnostic trouble code
	void clearDTC();
	// get battery voltage (works without ECU)
	float getVoltage();
	/**
	 * Read VIN from OBD and fill buffer with it (zero-ended only if success).
	 *
	 * :return:
	 * 		true if read successful
	 * 		NOTE: buffer modified always.
	 *
     * Buffer size should be >= OBD_RECV_BUF_SIZE const,
     * which was eradicated from code on 604d83caef (Nov 2015),
     * had started in 36a19a65 (Feb 2014) as 80,
     * increased to 128 on ae56f0682 (Apr 2014).
	 */
	bool getVIN(char* buffer, byte bufsize);
	// read OBFCM data (each pid has multiply data)
	bool GetOBFCM(DS_CAN_MSG* obfcmDataArray);
	// determine if the PID is supported
	bool isValidPID(byte pid);
	// specify custom CAN header ID
	void setHeaderID(uint32_t num);
	// toggle CAN sniffing mode, call setHeaderFilter and setHeaderMask before start sniffing
	void sniff(bool enabled = true);
	// set CAN bus header filter
	void setHeaderFilter(uint32_t num);
	// set CAN bus header filter bitmask
	void setHeaderMask(uint32_t bitmask);
	// receive sniffed data
	int receiveData(byte* buf, int len);
	// set current PID mode
	byte dataMode = 1;
	// occurrence of errors
	byte errors = 0;
	// bit map of supported PIDs
	byte pidmap[4 * 8] = {0};
	// link object pointer
	CLink* link = 0;
	/* Debug-aid incremented during `init()`. */
	int init_stage = -1;
	/* Discovered with `ATDP` cmd affter `init()` success. */
	std::string active_protocol;
protected:
	virtual void idleTasks() { delay(5); }
	char* getResponse(byte& pid, char* buffer, byte bufsize);
	uint8_t getPercentageValue(char* data);
	uint16_t getLargeValue(char* data);
	uint8_t getSmallValue(char* data);
	int16_t getTemperatureValue(char* data);
	int normalizeData(uint16_t id, char* data);
	byte checkErrorMessage(const char* buffer);
	char* getResultValue(char* buf);
	OBD_STATES m_state = OBD_DISCONNECTED;
};

#endif
