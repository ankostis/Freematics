/*************************************************************************
* Arduino Library for Freematics ONE+
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2012-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsOBD.h"

int dumpLine(char* buffer, int len)
{
	int bytesToDump = len >> 1;
	for (int i = 0; i < len; i++) {
		// find out first line end and discard the first line
		if (buffer[i] == '\r' || buffer[i] == '\n') {
			// go through all following \r or \n if any
			while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
			bytesToDump = i;
			break;
		}
	}
	memmove(buffer, buffer + bytesToDump, len - bytesToDump);
	return bytesToDump;
}

uint32_t hex2uint32(const char *p)
{
	char c = *p;
	uint32_t i = 0;
	for (uint8_t n = 0; c && n < 8; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && (n == 2 || n == 4 || n == 6)) {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (uint8_t n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && n == 2) {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

byte hex2uint8(const char *p)
{
	byte c1 = *p;
	byte c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >= 'a' && c1 <= 'f')
		c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 == 0)
		return (c1 & 0xf);
	else if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
		c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

/*************************************************************************
* OBD-II UART Bridge
*************************************************************************/

bool COBD::readPID(byte pid, int& result)
{
	char buffer[64];
	char* data = 0;
	sprintf(buffer, "%02X%02X\r", dataMode, pid);
	link->send(buffer);
	idleTasks();
	int ret = link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	if (ret > 0 && !checkErrorMessage(buffer)) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
			p += 3;
			byte curpid = hex2uint8(p);
			if (curpid == pid) {
				errors = 0;
				while (*p && *p != ' ') p++;
				while (*p == ' ') p++;
				if (*p) {
					data = p;
					break;
				}
			}
		}
	}

	if (!data) {
		errors++;
		return false;
	}
	result = normalizeData(pid, data);
	return true;
}

byte COBD::readPID(const byte pid[], byte count, int result[])
{
	byte results = 0;
	for (byte n = 0; n < count; n++) {
		if (readPID(pid[n], result[n])) {
			results++;
		}
	}
	return results;
}

bool COBD::readPIDMulti(DS_CAN_MSG* obdDataMulti)
{
	char command[64];
	char buffer[64];
    char data[16];
  	byte i = 0;
    byte nrOfChForMsg;
    byte idx_i = 0;

	while (obdDataMulti[i].idx){

		sprintf(command, "%02d%02X\r", obdDataMulti[i].service, obdDataMulti[i].pid);
		link->send(command);
		idleTasks();
		int ret = link->receive(buffer, 64, OBD_TIMEOUT_SHORT);
		if (ret > 0 /* && !checkErrorMessage(buffer)*/) {
			hex2uint8(buffer);
			char *p = buffer;
			if ((p = strstr(p, "41 "))) {
				p += 3;
				byte curpid = hex2uint8(p);
				if (curpid == obdDataMulti[i].pid) {
					p += 3;
					nrOfChForMsg = obdDataMulti[i].nrOfChForMsg;
					for (byte j=0; j < nrOfChForMsg; j++){
						idx_i = 0;
						for(byte k = 0; k < (obdDataMulti[i].length / 8); k++){
							while (*p && *p != ' '){
									data[idx_i] = *p;
									idx_i++;
								p++;
							}
							while (*p == ' '){
									data[idx_i] = *p;
									idx_i++;
								p++;
							}
						}
						obdDataMulti[i].value = normalizeData(obdDataMulti[i].id, data);//(hex2uint16(data)*obdDataMulti[i].gain + obdDataMulti[i].offset);
						i++;
					}
				} else {
				for (int j=0; j < obdDataMulti[i].nrOfChForMsg; j++) i++;
				}
			} else {
				for (int j=0; j < obdDataMulti[i].nrOfChForMsg; j++) i++;
			}
		}
	}
	return true;
}

int COBD::readDTC(uint16_t codes[], byte maxCodes)
{
	/*
	Response example:
	0: 43 04 01 08 01 09
	1: 01 11 01 15 00 00 00
	*/
	int codesRead = 0;
 	for (int n = 0; n < 6; n++) {
		char buffer[128];
		sprintf(buffer, n == 0 ? "03\r" : "03%02X\r", n);
		link->send(buffer);
		if (link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) > 0) {
			if (!strstr(buffer, "NO DATA")) {
				char *p = strstr(buffer, "43");
				if (p) {
					while (codesRead < maxCodes && *p) {
						p += 6;
						if (*p == '\r') {
							p = strchr(p, ':');
							if (!p) break;
							p += 2;
						}
						uint16_t code = hex2uint16(p);
						if (code == 0) break;
						codes[codesRead++] = code;
					}
				}
				break;
			}
		}
	}
	return codesRead;
}

void COBD::clearDTC()
{
	char buffer[32];
	link->send("04\r");
	link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG);
}

int COBD::normalizeData(uint16_t id, char* data)
{
	int result;
	switch (id) {
	case PID_RPM_ID:
	case PID_EVAP_SYS_VAPOR_PRESSURE_ID: // kPa
		result = getLargeValue(data); //getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE_ID: // kPa
		result = getSmallValue(data); //getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP_ID:
	case PID_INTAKE_TEMP_ID:
	case PID_AMBIENT_TEMP_ID:
	case PID_ENGINE_OIL_TEMP_ID:
		result = getSmallValue(data); //getTemperatureValue(data);
		break;
	case PID_THROTTLE_ID:
	case PID_COMMANDED_EGR_ID:
	case PID_COMMANDED_EVAPORATIVE_PURGE_ID:
	case PID_FUEL_LEVEL_ID:
	case PID_RELATIVE_THROTTLE_POS_ID:
	case PID_ABSOLUTE_THROTTLE_POS_B_ID:
	case PID_ABSOLUTE_THROTTLE_POS_C_ID:
	case PID_ACC_PEDAL_POS_D_ID:
	case PID_ACC_PEDAL_POS_E_ID:
	case PID_ACC_PEDAL_POS_F_ID:
	case PID_COMMANDED_THROTTLE_ACTUATOR_ID:
	case PID_ENGINE_LOAD_ID:
	case PID_ABSOLUTE_ENGINE_LOAD_ID:
	case PID_ETHANOL_FUEL_ID:
	case PID_HYBRID_BATTERY_PERCENTAGE_ID:
		result = getSmallValue(data); //getPercentageValue(data);
		break;
	case PID_MAF_FLOW_ID: // grams/sec
		result = getLargeValue(data); //getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE_ID:
		result = getSmallValue(data); //(int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE_ID: // km
	case PID_DISTANCE_WITH_MIL_ID: // km
	case PID_TIME_WITH_MIL_ID: // minute
	case PID_TIME_SINCE_CODES_CLEARED_ID: // minute
	case PID_RUNTIME_ID: // second
	case PID_FUEL_RAIL_PRESSURE_ID: // kPa
	case PID_ENGINE_REF_TORQUE_ID: // Nm
		result = getLargeValue(data);
		break;
	case PID_CONTROL_MODULE_VOLTAGE_ID: // V
		result = getLargeValue(data); //getLargeValue(data) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE_ID: // L/h
		result = getLargeValue(data); //getLargeValue(data) / 20;
		break;
	case MULTIPID_ENGINE_FUEL_RATE_GS_ID: // g/s
		result = getLargeValue(data); //getLargeValue(data) / 5;
		break;
	case MULTIPID_VEHICLE_FUEL_RATE_GS_ID: // g/s
		result = getLargeValue(data); //getLargeValue(data) / 5;
		break;
	case PID_ENGINE_TORQUE_DEMANDED_ID: // %
	case PID_ENGINE_TORQUE_PERCENTAGE_ID: // %
		result = getSmallValue(data); //(int)getSmallValue(data) - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1_ID:
	case PID_LONG_TERM_FUEL_TRIM_1_ID:
	case PID_SHORT_TERM_FUEL_TRIM_2_ID:
	case PID_LONG_TERM_FUEL_TRIM_2_ID:
	case PID_EGR_ERROR_ID:
		result = getSmallValue(data); //((int)getSmallValue(data) - 128) * 100 / 128;
		break;
	case PID_FUEL_INJECTION_TIMING_ID:
		result = getLargeValue(data); //((int32_t)getLargeValue(data) - 26880) / 128;
		break;
	case PID_CATALYST_TEMP_B1S1_ID:
	case PID_CATALYST_TEMP_B2S1_ID:
	case PID_CATALYST_TEMP_B1S2_ID:
	case PID_CATALYST_TEMP_B2S2_ID:
		result = getLargeValue(data); //getLargeValue(data) / 10 - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO_ID: // 0~200
		result = getLargeValue(data); //(long)getLargeValue(data) * 200 / 65536;
		break;
	default:
		result = getSmallValue(data);
	}
	return result;
}

char* COBD::getResponse(byte& pid, char* buffer, byte bufsize)
{
	while (link->receive(buffer, bufsize, OBD_TIMEOUT_SHORT) > 0) {
		char *p = buffer;
		while ((p = strstr(p, "41 "))) {
		    p += 3;
		    byte curpid = hex2uint8(p);
		    if (pid == 0) pid = curpid;
		    if (curpid == pid) {
		        errors = 0;
		        p += 2;
		        if (*p == ' ')
		            return p + 1;
		    }
		}
	}
	return 0;
}

void COBD::enterLowPowerMode()
{
  	char buf[32];
	link->sendCommand("ATLP\r", buf, sizeof(buf), 1000);
}

void COBD::leaveLowPowerMode()
{
	// send any command to wake up
	char buf[32];
	for (byte n = 0; n < 30 && !link->sendCommand("ATI\r", buf, sizeof(buf), 1000); n++);
}

char* COBD::getResultValue(char* buf)
{
	char* p = buf;
	for (;;) {
		if (isdigit(*p) || *p == '-') {
			return p;
		}
		p = strchr(p, '\r');
		if (!p) break;
		if (*(++p) == '\n') p++;
	}
	return 0;
}

float COBD::getVoltage()
{
    char buf[32];
	if (link->sendCommand("ATRV\r", buf, sizeof(buf), 500) > 0) {
		char* p = getResultValue(buf);
		if (p) return (float)atof(p);
    }
    return 0;
}

bool COBD::getVIN(char* buffer, byte bufsize)
{
	for (byte n = 0; n < 2; n++) {
		if (link->sendCommand("0902\r", buffer, bufsize, OBD_TIMEOUT_LONG)) {
			int len = hex2uint16(buffer);
			char *p = strstr(buffer + 4, "0: 49 02 01");
			if (p) {
				char *q = buffer;
				p += 11; // skip the header
				do {
					while (*(++p) == ' ');
					for (;;) {
						*(q++) = hex2uint8(p);
						while (*p && *p != ' ') p++;
						while (*p == ' ') p++;
						if (!*p || *p == '\r') break;
					}
					p = strchr(p, ':');
				} while(p);
				*q = 0;
				if (q - buffer == len - 3) {
					return true;
				}
			}
		}
		delay(100);
	}
    return false;
}

bool COBD::GetOBFCM (DS_CAN_MSG* obfcmDataArray)
{
  // Write C++ code here
	char command[128];
  	char data[16];
  	byte idx = 0;
  	byte idx2 = 0;
	byte msgNr = 0;
	char buffer[128];
	byte bufsize;

	while(obfcmDataArray[idx2].idx){
		sprintf(command, "%02d%02X\r", obfcmDataArray[idx2].service, obfcmDataArray[idx2].pid);
		bufsize = sizeof(buffer);
		if (link->sendCommand(command, buffer, bufsize, OBD_TIMEOUT_LONG))
		{
			int len = hex2uint16(buffer);
			if (len >= 19){
				char *p = buffer+4;
				if (p) {
					p += 12; // skip the header

					msgNr = obfcmDataArray[idx2].nrOfChForMsg;
					for(byte k = 0; k < msgNr; k++){
						idx = 0;
						for (byte n = 0; n < 4; n++){
							while (*p && *p != ' '){
								data[idx] = *p;
								idx++;
								p++;
							}
							while (*p == ' '){
								data[idx] = *p;
								idx++;
								p++;
							}

							if (*p == '\r'){
								p = strchr(p, ':');
								p += 2;
							}
						}
/*					if (!data) {
							errors++;
							return false;
						}
*/
						obfcmDataArray[idx2].value = hex2uint32(data); //(hex2uint32(data)*obfcmDataArray[idx2].gain + obfcmDataArray[idx2].offset);
						idx2++;
					}
				}
			}
			else
				return true;
		}
	}
	return true;
}

bool COBD::isValidPID(byte pid)
{
	pid--;
	byte i = pid >> 3;
	byte b = 0x80 >> (pid & 0x7);
	return true || (pidmap[i] & b) != 0;
}

bool COBD::init(
	OBD_PROTOCOLS protocol,
	std::vector<std::vector<std::string>> obd_alt_init_cmds
) {
	ESP_LOGI(TAG_OBD, "<init> proto: %i", protocol);
	const char *initcmd[] = {"ATE0\r", "ATH0\r"};
	char buffer[64];

	if (!link) {
		return false;
	}

	m_state = OBD_DISCONNECTED;

	init_stage = 0;
	for (byte n = 0; n < 10; n++) {
		if (link->sendCommand("ATZ\r", buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
			init_stage += 1;
			goto success_1;
		}
	}
	return false;
success_1:

	for (byte i = 0; i < sizeof_array(initcmd); i++) {
		link->sendCommand(initcmd[i], buffer, sizeof(buffer), OBD_TIMEOUT_SHORT);
	}
	init_stage += 1;

	if (protocol != PROTO_AUTO) {
		sprintf(buffer, "ATSP %X\r", protocol);
		if (!link->sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) || !strstr(buffer, "OK")) {
			// Bail-out, set-protocol  command must not fail.
			return false;
		}
		init_stage += 1;
		if (protocol == PROTO_J1939) {
			m_state = OBD_CONNECTED;
			errors = 0;
			return true;
		}
	}

	for(auto &cmd_list : obd_alt_init_cmds) {
		for (int n = 0; n < 2; n++) {
			int _value;
			if (readPID(PID_SPEED, _value)) {
				init_stage += 10;
				goto success;
			}
		}

		// Send Alternate Init Commands
		for(auto &cmd : cmd_list) {
			sprintf(buffer, "%s\r", cmd.c_str());
			if (!link->sendCommand(buffer, buffer, sizeof(buffer), OBD_TIMEOUT_SHORT) ||
					!strstr(buffer, "OK")) {
				// NOTE: we should directly send the next `cmd_list`
				// without trying to read PID_SPEED, but...
				break;
			}

		}
	}
	return false;

success:

	// Read Protocol.
	sprintf(buffer, "ATDP\r");
	link->send(buffer);
	if (link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_SHORT)) {
		active_protocol = buffer;
		init_stage += 1;
	}

	// load pid map
	memset(pidmap, 0xff, sizeof(pidmap));
	for (int i = 0; i < 8; i++) {
		byte pid = i * 0x20;
		sprintf(buffer, "%02X%02X\r", dataMode, pid);
		link->send(buffer);
		if (!link->receive(buffer, sizeof(buffer), OBD_TIMEOUT_LONG) || checkErrorMessage(buffer)) {
			init_stage += 1;
			break;
		}
		for (char *p = buffer; (p = strstr(p, "41 ")); ) {
			p += 3;
			if (hex2uint8(p) == pid) {
				p += 2;
				for (byte n = 0; n < 4 && *(p + n * 3) == ' '; n++) {
					pidmap[i * 4 + n] = hex2uint8(p + n * 3 + 1);
				}
			}
		}
	}
	m_state = OBD_CONNECTED;
	errors = 0;
	return true;
}

void COBD::reset()
{
	ESP_LOGD(TAG_OBD, "<reset>");
	char buf[32];
	link->sendCommand("ATR\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

void COBD::uninit()
{
	ESP_LOGI(TAG_OBD, "<down>");
	char buf[32];
	link->sendCommand("ATPC\r", buf, sizeof(buf), OBD_TIMEOUT_SHORT);
}

byte COBD::checkErrorMessage(const char* buffer)
{
	const char *errmsg[] = {"UNABLE", "ERROR", "TIMEOUT", "NO DATA"};
	for (byte i = 0; i < sizeof_array(errmsg); i++) {
		if (strstr(buffer, errmsg[i])) return i + 1;
	}
	return 0;
}

uint8_t COBD::getPercentageValue(char* data)
{
  return (uint16_t)hex2uint8(data) * 100 / 255;
}

uint16_t COBD::getLargeValue(char* data)
{
  return hex2uint16(data);
}

uint8_t COBD::getSmallValue(char* data)
{
  return hex2uint8(data);
}

int16_t COBD::getTemperatureValue(char* data)
{
  return (int)hex2uint8(data) - 40;
}

void COBD::setHeaderID(uint32_t num)
{
	char buf[32];
	sprintf(buf, "ATSH %X\r", num & 0xffffff);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
	sprintf(buf, "ATCP %X\r", num & 0x1f);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}

void COBD::sniff(bool enabled)
{
	char buf[32];
	link->sendCommand(enabled ? "ATM1\r" : "ATM0\r", buf, sizeof(buf), 1000);
}

void COBD::setHeaderFilter(uint32_t num)
{
	char buf[32];
	sprintf(buf, "ATCF %X\r", num);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}

void COBD::setHeaderMask(uint32_t bitmask)
{
	char buf[32];
	sprintf(buf, "ATCM %X\r", bitmask);
	link->sendCommand(buf, buf, sizeof(buf), 1000);
}

int COBD::receiveData(byte* buf, int len)
{
	int n = 0;
	for (n = 0; n < len; ) {
		int c = link->read();
		if (c == -1 || c == '\r') break;
		buf[n++] = c;
	}
	if (n == 0) return 0;
	int bytes = 0;
	len = n;
	if (buf[0] == '$') {
		for (n = 1; n < len && buf[n] != ','; n++);
		for (; n < len && buf[n] == ','; bytes++) {
			byte d = hex2uint8((const char*)buf + n + 1);
			n += 3;
			if (buf[n] != ',' && buf[n] != '\r') {
				if (d != hex2uint8((const char*)buf + n)) break;
				n += 2;
			}
			buf[bytes] = d;
		}
	} else {
		for (n = 0; n < len; bytes++) {
			buf[bytes] = hex2uint8((const char*)buf + n);
			n += 2;
			if (buf[n++] != ' ') break;
		}
	}

	ESP_LOGD(TAG_OBD, "<RECV> x%i, |%.*s|", bytes, bytes, buf);
	return bytes;
}
