/*************************************************************************
* Freematics MEMS motion sensor helper classes
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2016-2020 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#ifndef FREEMATICS_MEMS
#define FREEMATICS_MEMS

#include "FreematicsBase.h"
#include "utility/ICM_20948_C.h"	// The C backbone

inline constexpr const char TAG_MEMS[] = "MEMS";

// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
// document; the MPU9250 and MPU9150 are virtually identical but the latter has
// a different register map

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2      0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A       0x10

#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define MOT_DUR           0x20
// Zero-motion detection threshold bits [7:0]
#define ZMOT_THR          0x21
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
#define ZRMOT_DUR         0x22

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250   0x75 // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS  0x0C   // Address of magnetometer

enum {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
#define Ascale AFS_2G
#define Gscale GFS_250DPS

#define mRes (10.*4912./32760.0)

#if Ascale == AFS_2G
  #define aRes (2.0/32768.0)
#elif Ascale == AFS_4G
  #define aRes (4.0/32768.0)
#elif Ascale == AFS_8G
  #define aRes (8.0/32768.0)
#elif Ascale == AFS_16G
  #define aRes (16.0/32768.0)
#endif

#if Gscale == GFS_250DPS
  #define gRes (250.0/32768.0)
#elif Gscale == GFS_500DPS
  #define gRes (500.0/32768.0)
#elif Gscale == GFS_1000DPS
  #define gRes (1000.0/32768.0)
#elif Gscale == GFS_2000DPS
  #define gRes (2000.0/32768.0)
#endif

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
#define Mmode 0x02

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

class CQuaterion
{
public:
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void getOrientation(ORIENTATION* ori);
private:
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
  // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
  float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
  uint32_t firstUpdate = 0; // used to calculate integration interval
  uint32_t lastUpdate = 0;
  float deltat = 0.0f;
};

class MEMS_I2C
{
public:
  MEMS_I2C() {};
  virtual ~MEMS_I2C() {};
  virtual byte begin(bool fusion = false) = 0;
  virtual void end() { uninitI2C(); }
  virtual bool read(float* acc, float* gyr = 0, float* mag = 0, float* temp = 0, ORIENTATION* ori = 0) = 0;
  virtual const char *name() = 0;
protected:
  bool initI2C(unsigned long clock);
  void uninitI2C();
};

class MPU9250 : public MEMS_I2C
{
public:
  byte begin(bool fusion = false);
  bool read(float* acc, float* gyr = 0, float* mag = 0, float* temp = 0, ORIENTATION* ori = 0);
  virtual const char* name() { return "MPU9250";  }
private:
  void writeByte(uint8_t, uint8_t);
  uint8_t readByte(uint8_t);
  bool readBytes(uint8_t, uint8_t, uint8_t *);
  void readAccelData(int16_t *);
  int16_t readTempData();
  void init();
  void getAres();
  void getMres();
  void getGres();
  void readGyroData(int16_t *);
  void readMagData(int16_t *);
  bool initAK8963(float *);
  void calibrateMPU9250(float * gyroBias, float * accelBias);
  void MPU9250SelfTest(float * destination);
  void writeByteAK(uint8_t, uint8_t);
  uint8_t readByteAK(uint8_t);
  bool readBytesAK(uint8_t, uint8_t, uint8_t *);
  float gyroBias[3] = {0};
  float accelBias[3] = {0};      // Bias corrections for gyro and accelerometer
  float magCalibration[3] = {0};
  int16_t accelCount[3] = {0};
  int16_t gyroCount[3] = {0};
  int16_t magCount[3] = {0};    // Stores the 16-bit signed magnetometer sensor output
  CQuaterion* quaterion = 0;
};

#define ICM_20948_ARD_UNUSED_PIN 0xFF

// Base
class ICM_20948 {
private:
protected:
    ICM_20948_Device_t  _device;
    bool                _has_magnetometer;

    float               getTempC            ( int16_t val );
    float               getGyrDPS           ( int16_t axis_val );
    float               getAccMG            ( int16_t axis_val );
    float               getMagUT            ( int16_t axis_val );

public:
    ICM_20948() {};
    virtual ~ICM_20948() {};

    ICM_20948_AGMT_t    agmt;                                                               // Acceleometer, Gyroscope, Magenetometer, and Temperature data
    ICM_20948_AGMT_t    getAGMT             ( void );                                        // Updates the agmt field in the object and also returns a copy directly

    float               magX                ( void );// micro teslas
    float               magY                ( void );// micro teslas
    float               magZ                ( void );// micro teslas

    float               accX                ( void );// milli g's
    float               accY                ( void );// milli g's
    float               accZ                ( void );// milli g's

    float               gyrX                ( void );// degrees per second
    float               gyrY                ( void );// degrees per second
    float               gyrZ                ( void );// degrees per second

    float               temp                ( void );// degrees celsius


    // Status from latest operation
    ICM_20948_Status_e  status;
    // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied
    const char*         statusString        ( ICM_20948_Status_e stat = ICM_20948_Stat_NUM );

    // Device Level
    ICM_20948_Status_e	setBank			    ( uint8_t bank );					            // Sets the bank
    ICM_20948_Status_e	swReset			    ( void );							            // Performs a SW reset
    ICM_20948_Status_e	sleep			    ( bool on = false );					        // Set sleep mode for the chip
    ICM_20948_Status_e	lowPower		    ( bool on = true );							    // Set low power mode for the chip
    ICM_20948_Status_e	setClockSource	    ( ICM_20948_PWR_MGMT_1_CLKSEL_e source );       // Choose clock source
    ICM_20948_Status_e	checkID			    ( void );								        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

    bool	            dataReady		    ( void );				                        // Returns 'true' if data is ready
    uint8_t             getWhoAmI		    ( void );								        // Return whoami in out prarmeter
    // Returns true if communications with the device are sucessful
    bool                isConnected         ( void );


    // Internal Sensor Options
    // Use to set accel, gyro, and I2C master into cycled or continuous modes
    ICM_20948_Status_e	setSampleMode	    ( uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode );
    ICM_20948_Status_e	setFullScale 	    ( uint8_t sensor_id_bm, ICM_20948_fss_t fss );
    ICM_20948_Status_e	setDLPFcfg		    ( uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg );
    ICM_20948_Status_e	enableDLPF		    ( uint8_t sensor_id_bm, bool enable );
    ICM_20948_Status_e	setSampleRate	    ( uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt );

    // Interrupts on INT and FSYNC Pins
    ICM_20948_Status_e  clearInterrupts         ( void );

    ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
    ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
    ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse
    ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
    ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
    ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

    ICM_20948_Status_e	intEnableI2C            ( bool enable );
    ICM_20948_Status_e	intEnableDMP            ( bool enable );
    ICM_20948_Status_e	intEnablePLL            ( bool enable );
    ICM_20948_Status_e	intEnableWOM            ( bool enable );
    ICM_20948_Status_e	intEnableWOF            ( bool enable );
    ICM_20948_Status_e	intEnableRawDataReady   ( bool enable );
    ICM_20948_Status_e	intEnableOverflowFIFO   ( uint8_t bm_enable );
    ICM_20948_Status_e	intEnableWatermarkFIFO  ( uint8_t bm_enable );

    // Interface Options
    ICM_20948_Status_e	i2cMasterPassthrough 	( bool passthrough = true );
    ICM_20948_Status_e	i2cMasterEnable         ( bool enable = true );
    ICM_20948_Status_e	i2cMasterConfigureSlave ( uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false );

    ICM_20948_Status_e 	i2cMasterSLV4Transaction( uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len, bool Rw, bool send_reg_addr = true );
    ICM_20948_Status_e	i2cMasterSingleW        ( uint8_t addr, uint8_t reg, uint8_t data );
    uint8_t 	        i2cMasterSingleR        ( uint8_t addr, uint8_t reg );


    // Default Setup
    ICM_20948_Status_e          startupDefault          ( void );
    virtual ICM_20948_Status_e  startupMagnetometer     ( void );
    virtual ICM_20948_Status_e  getMagnetometerData     ( ICM_20948_AGMT_t* pagmt );


    // direct read/write
    ICM_20948_Status_e  read                ( uint8_t reg, uint8_t* pdata, uint32_t len);
    ICM_20948_Status_e  write               ( uint8_t reg, uint8_t* pdata, uint32_t len);

    CQuaterion* quaterion = 0;
};

class ICM_20948_I2C : public MEMS_I2C, public ICM_20948 {
public:
    uint8_t                 _addr;
    uint8_t                 _ad0;
    bool                    _ad0val;
    ICM_20948_Serif_t       _serif;

    virtual const char* name() { return "ICM_20948";  }

    virtual ICM_20948_Status_e  readMag( uint8_t reg, uint8_t* pdata, uint8_t len );
    virtual ICM_20948_Status_e  writeMag( uint8_t reg, uint8_t* pdata, uint8_t len );

    byte begin(bool fusion = false);
    bool read(float* acc, float* gyr = 0, float* mag = 0, float* tmp = 0, ORIENTATION* ori = 0);

    ICM_20948_Status_e  startupMagnetometer( void );
    ICM_20948_Status_e  magWhoIAm( void );
    bool                magIsConnected( void );
    ICM_20948_Status_e  getMagnetometerData     ( ICM_20948_AGMT_t* pagmt );
};

MEMS_I2C* init_MEMS(bool enable_orientation);

#endif  // FREEMATICS_MEMS
