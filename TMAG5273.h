#ifndef TMAG5273_H
#define TMAG5273_H

#include "Arduino.h"
#include <Wire.h>

// Register definition

#define DEVICE_CONFIG_1 0x00
#define DEVICE_CONFIG_2 0x01
#define SENSOR_CONFIG_1 0x02
#define SENSOR_CONFIG_2 0x03
#define X_THR_CONFIG 0x04
#define Y_THR_CONFIG 0x05
#define Z_THR_CONFIG 0x06
#define T_CONFIG 0x07
#define INT_CONFIG_1 0x08
#define MAG_GAIN_CONFIG 0x09
#define MAG_OFFSET_CONFIG_1 0x0A
#define MAG_OFFSET_CONFIG_2 0x0B
#define I2C_ADDRESS 0x0C
#define DEVICE_ID 0x0D
#define MANUFACTURER_ID_LSB 0x0E
#define MANUFACTURER_ID_MSB 0x0F
#define T_MSB_RESULT 0x10
#define T_LSB_RESULT 0x11
#define X_MSB_RESULT 0x12
#define X_LSB_RESULT 0x13
#define Y_MSB_RESULT 0x14
#define Y_LSB_RESULT 0x15
#define Z_MSB_RESULT 0x16
#define Z_LSB_RESULT 0x17
#define CONV_STATUS 0x18
#define ANGLE_RESULT_MSB 0x19
#define ANGLE_RESULT_LSB 0x1A
#define MAGNITUDE_RESULT 0x1B
#define DEVICE_STATUS 0x1C


#define REG_DEVICE_CONFIG_1_DEFAULT 0x00
#define REG_DEVICE_CONFIG_2_DEFAULT 0x00
#define REG_SENSOR_CONFIG_1_DEFAULT 0x74
#define REG_SENSOR_CONFIG_2_DEFAULT 0x03
#define REG_X_THR_CONFIG_DEFAULT 0x00
#define REG_Y_THR_CONFIG_DEFAULT 0x00
#define REG_Z_THR_CONFIG_DEFAULT 0x00
#define REG_T_CONFIG_DEFAULT 0x00
#define REG_INT_CONFIG_1_DEFAULT 0x01
#define REG_MAG_GAIN_CONFIG_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_1_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_2_DEFAULT 0x00

#define TMAG5273_DEFAULT_ADDR 0x35
#define TMAG5273_ARRAY_START_ADDR 0x36

#define TSENSET0 25.
#define TADCRES 60.1
#define TADCT0 17508

// conv time upper bound? from datasheet
#define T_START_MEASURE 70   // set to zero seems ok
#define T_MEASURE_CHANNEL 25   // might be a little smaller but may cause imcomplete conversion
#define T_MEASURE_DUMMY 0
#define CONV_TIME_FROM_AVG_MODE(temp_en, mode) (T_START_MEASURE + (temp_en + 3) * T_MEASURE_CHANNEL * (1 + (1 << (mode >> 2))) + T_MEASURE_DUMMY) 

#define ARRAY_SINGLE_RC_DELAY_MS 1000
#define ARRAY_POWER_UP_TIMEOUT_MS 500

typedef enum tmag5273_operating_mode {
    TMAG5273_OPERATING_MODE_STANDBY = 0x0,
    TMAG5273_OPERATING_MODE_SLEEP,
    TMAG5273_OPERATING_MODE_MEASURE,
    TMAG5273_OPERATING_MODE_WS
} tmag5273_operating_mode_t;

typedef enum tmag5273_read_mode {
    TMAG5273_READ_MODE_STANDARD = 0x0,
    TMAG5273_READ_MODE_SENSOR16,
    TMAG5273_READ_MODE_SENSOR8
} tmag5273_read_mode_t;

typedef enum tmag5273_mag_range {
    TMAG5273_MAG_RANGE_40MT = 40,     // for TMAG5273A1
    TMAG5273_MAG_RANGE_80MT = 80,     // for TMAG5273A1
    TMAG5273_MAG_RANGE_133MT = 133,   // for TMAG5273A2
    TMAG5273_MAG_RANGE_266MT = 266    // for TMAG5273A2
} tmag5273_mag_range_t;

typedef enum tmag5273_mag_range_multiple {
    TMAG5273_MAG_RANGE_MULTIPLE_1x = 0x0, 
    TMAG5273_MAG_RANGE_MULTIPLE_2x = 0x3
} tmag5273_mag_range_multiple_t;

typedef enum tmag5273_lp_ln_mode { 
    TMAG5273_LOW_ACTIVE_CURRENT = 0x0,
    TMAG5273_LOW_NOISE = 0x1 << 4
} tmag5273_lp_ln_mode_t;

typedef enum tmag5273_mag_tempco_mode { 
    TMAG5273_NO_MAG_TEMPCO = 0x0,
    TMAG5273_MAG_TEMPCO_NdBFe = 0x1 << 5,
    TMAG5273_MAG_TEMPCO_FERRITE = 0x3 << 5
} tmag5273_mag_tempco_mode_t;

typedef enum tmag5273_conv_avg_mode { 
    TMAG5273_CONV_AVG_1X = 0x0,
    TMAG5273_CONV_AVG_2X = 0x1 << 2,
    TMAG5273_CONV_AVG_4X = 0x2 << 2,
    TMAG5273_CONV_AVG_8X = 0x3 << 2,
    TMAG5273_CONV_AVG_16X = 0x4 << 2,
    TMAG5273_CONV_AVG_32X = 0x5 << 2,
} tmag5273_conv_avg_mode_t;

typedef enum tmag5273_temp_ch_en { // only decides the TMAG5273_READ_MODE_SENSOR16 data
    TMAG5273_TEMP_CH_DISABLED = 0x0,
    TMAG5273_TEMP_CH_ENABLED
} tmag5273_temp_ch_en_t;


// TODO support trigger

class TMAG5273 {
public:
    TMAG5273(TwoWire *wire);
    void init(void);
    void initAll(void);

    void switchSensor(uint8_t addr);
    void printDeviceTable(HardwareSerial* serial);

    void waitSensorArrayOff(uint32_t rc_delay=ARRAY_SINGLE_RC_DELAY_MS);
    uint8_t initSensorArray(uint32_t timeout=ARRAY_POWER_UP_TIMEOUT_MS);
    uint8_t readSensorArray(float* data_ptr);

    void modifyI2CAddress(uint8_t new_addr);

    float readTemperature(void);
    uint8_t readMagneticField(float* Bx, float* By, float* Bz);
    uint8_t readMagneticField(float* Bx, float* By, float* Bz, float* T);

    // config the setting before initialization
    void configOperatingMode(tmag5273_operating_mode mode);
    void configReadMode(tmag5273_read_mode mode);
    void configMagRange(tmag5273_mag_range range);
    void configLplnMode(tmag5273_lp_ln_mode mode);
    void configMagTempcoMode(tmag5273_mag_tempco_mode mode);
    void configConvAvgMode(tmag5273_conv_avg_mode mode);
    void configTempChEnabled(bool enabled);

private:
    TwoWire *i2c_dev = NULL;
    int8_t currentDeviceAddress = TMAG5273_DEFAULT_ADDR;
    float _temp;
    uint8_t arrayDevices = 0;

    tmag5273_mag_tempco_mode magTempcoMode = TMAG5273_NO_MAG_TEMPCO;
    tmag5273_conv_avg_mode convAvgMode = TMAG5273_CONV_AVG_1X;
    uint16_t estiConversionTime = CONV_TIME_FROM_AVG_MODE(TMAG5273_TEMP_CH_DISABLED, TMAG5273_CONV_AVG_1X);
    tmag5273_read_mode readMode = TMAG5273_READ_MODE_STANDARD;
    
    tmag5273_lp_ln_mode lplnMode = TMAG5273_LOW_ACTIVE_CURRENT;
    tmag5273_operating_mode operatingMode = TMAG5273_OPERATING_MODE_STANDBY;
    
    tmag5273_mag_range_multiple magRangeMultiple = TMAG5273_MAG_RANGE_MULTIPLE_1x;
    tmag5273_temp_ch_en tempChEn = TMAG5273_TEMP_CH_DISABLED;
    
    float magRangeValue = (float) TMAG5273_MAG_RANGE_40MT;

    void readRegister(uint8_t reg, uint8_t *data);
    void readRegister(uint8_t reg, int16_t *data);

    bool writeRegister(uint8_t reg, uint8_t data);
    bool writeRegisterGeneral(uint8_t reg, uint8_t data);

    uint8_t _readMagneticField(float* Bx, float* By, float* Bz, float* T);
};

#endif /* TMAG5273_H */
