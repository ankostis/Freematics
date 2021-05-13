// Mode 1 PIDs
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05
#define PID_SHORT_TERM_FUEL_TRIM_1 0x06
#define PID_LONG_TERM_FUEL_TRIM_1 0x07
#define PID_SHORT_TERM_FUEL_TRIM_2 0x08
#define PID_LONG_TERM_FUEL_TRIM_2 0x09
#define PID_FUEL_PRESSURE 0x0A
#define PID_INTAKE_MAP 0x0B
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_TIMING_ADVANCE 0x0E
#define PID_INTAKE_TEMP 0x0F
#define PID_MAF_FLOW 0x10
#define PID_THROTTLE 0x11
#define PID_AUX_INPUT 0x1E
#define PID_RUNTIME 0x1F
#define PID_DISTANCE_WITH_MIL 0x21
#define PID_COMMANDED_EGR 0x2C
#define PID_EGR_ERROR 0x2D
#define PID_COMMANDED_EVAPORATIVE_PURGE 0x2E
#define PID_FUEL_LEVEL 0x2F
#define PID_WARMS_UPS 0x30
#define PID_DISTANCE 0x31
#define PID_EVAP_SYS_VAPOR_PRESSURE 0x32
#define PID_BAROMETRIC 0x33
#define PID_CATALYST_TEMP_B1S1 0x3C
#define PID_CATALYST_TEMP_B2S1 0x3D
#define PID_CATALYST_TEMP_B1S2 0x3E
#define PID_CATALYST_TEMP_B2S2 0x3F
#define PID_CONTROL_MODULE_VOLTAGE 0x42
#define PID_ABSOLUTE_ENGINE_LOAD 0x43
#define PID_AIR_FUEL_EQUIV_RATIO 0x44
#define PID_RELATIVE_THROTTLE_POS 0x45
#define PID_AMBIENT_TEMP 0x46
#define PID_ABSOLUTE_THROTTLE_POS_B 0x47
#define PID_ABSOLUTE_THROTTLE_POS_C 0x48
#define PID_ACC_PEDAL_POS_D 0x49
#define PID_ACC_PEDAL_POS_E 0x4A
#define PID_ACC_PEDAL_POS_F 0x4B
#define PID_COMMANDED_THROTTLE_ACTUATOR 0x4C
#define PID_TIME_WITH_MIL 0x4D
#define PID_TIME_SINCE_CODES_CLEARED 0x4E
#define PID_ETHANOL_FUEL 0x52
#define PID_FUEL_RAIL_PRESSURE 0x59
#define PID_HYBRID_BATTERY_PERCENTAGE 0x5B
#define PID_ENGINE_OIL_TEMP 0x5C
#define PID_FUEL_INJECTION_TIMING 0x5D
#define PID_ENGINE_FUEL_RATE 0x5E
#define PID_ENGINE_TORQUE_DEMANDED 0x61
#define PID_ENGINE_TORQUE_PERCENTAGE 0x62
#define PID_ENGINE_REF_TORQUE 0x63
#define PID_ENGINE_FUEL_RATE_GS 0x9D

// Mode 9 PIDs
#define OBFCM_MAX_CHANNELS																		6			// PID 0x1A, 0x1C
#define OBFCM_TOTAL_DISTANCE_TRAVELED_RECENT 									0x17	// idx = 1
#define OBFCM_TOTAL_DISTANCE_TRAVELED_LIFETIME 								0x17	// idx = 2
#define OBFCM_TOTAL_FUEL_TRAVELED_RECENT											0x17	// idx = 3
#define OBFCM_TOTAL_FUEL_TRAVELED_LIFETIME										0x17	// idx = 4
#define OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_RECENT							0x1A	// idx = 5
#define OBFCM_PEV_DIST_CHARGE_DEPL_ENG_OFF_LIFETIME						0x1A	// idx = 6
#define OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_RECENT							0x1A	// idx = 7
#define OBFCM_PEV_DIST_CHARGE_DEPL_ENG_ON_LIFETIME						0x1A	// idx = 8
#define OBFCM_PEV_DIST_CHARGE_INCREASING_RECENT								0x1A	// idx = 9
#define OBFCM_PEV_DIST_CHARGE_INCREASING_LIFETIME							0x1A	// idx = 10
#define OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_RECENT						0x1B	// idx = 11
#define OBFCM_PEV_FUEL_CONSUMED_CHARGE_DEPL_LIFETIME					0x1B	// idx = 12
#define OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_RECENT						0x1B	// idx = 13
#define OBFCM_PEV_FUEL_CONSUMED_CHARGE_INCR_LIFETIME					0x1B	// idx = 14
#define OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_RECENT			0x1C	// idx = 15
#define OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_OFF_LIFETIME		0x1C	// idx = 16
#define OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_RECENT				0x1C	// idx = 17
#define OBFCM_PEV_GRID_ENERGY_CHARGE_DEPL_ENG_ON_LIFETIME			0x1C	// idx = 18
#define OBFCM_PEV_GRID_ENERGY_IN_BATTERY_RECENT								0x1C	// idx = 19
#define OBFCM_PEV_GRID_ENERGY_IN_BATTERY_LIFETIME							0x1C	// idx = 20

#define MULTIPID_ENGINE_FUEL_RATE_GS 0x9D
#define MULTIPID_VEHICLE_FUEL_RATE_GS 0x9D



typedef struct
{
	byte		idx;						// idx che identifica il PID nella struttura dati
	byte		pid;						// PID: Parameter ID per OBD
	byte		nrOfChForMsg;		// numero di canali che compongono il messaggio, ovvero lo stesso PID
	byte		curMsg;					// canale corrente (da 1 a NrOfChForMsg)
	byte		service;				// standard diagnostic service: 0x01..0x0A; oppure custom service: 0x22
	byte		startBit;				// bit inizio messaggio
	byte		length;					// lunghezza in bit
	float			gain;					  // fattore moltiplicativo della funzione di trasferimento
	float			offset;					// fattore additivo della funzione di trasferimento
	float			value;					// valore ingegnerizzato
}
DS_CAN_MSG;


typedef enum {
    PROTO_AUTO = 0x0,
    PROTO_ISO_9141_2 = 0x3,
    PROTO_KWP2000_5KBPS = 0x4,
    PROTO_KWP2000_FAST = 0x5,
    PROTO_CAN_11B_500K = 0x6,
    PROTO_CAN_29B_500K = 0x7,
    PROTO_CAN_29B_250K = 0x8,
    PROTO_CAN_11B_250K = 0x9,
    PROTO_J1939 = 0xB,
} OBD_PROTOCOLS;

// states
typedef enum {
    OBD_DISCONNECTED = 0,
    OBD_CONNECTING = 1,
    OBD_CONNECTED = 2,
    OBD_FAILED = 3
} OBD_STATES;
