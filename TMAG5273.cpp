#include "TMAG5273.h"


TMAG5273::TMAG5273() {}

void TMAG5273::begin(TwoWire *wire) {
    i2c_dev = wire;
    initAll();
}

void TMAG5273::configOperatingMode(tmag5273_operating_mode mode)
{
    operatingMode = mode;
}

void TMAG5273::configReadMode(tmag5273_read_mode mode)
{
    readMode = mode;
}

void TMAG5273::configMagRange(tmag5273_mag_range range)
{
    magRange = range; 
    if (magRange == TMAG5273_MAG_RANGE_40MT) magRangeValue = 40.f;
    else magRangeValue = 80.f;
}

void TMAG5273::configLplnMode(tmag5273_lp_ln_mode mode)
{
    lplnMode = mode;
}

void TMAG5273::configMagTempcoMode(tmag5273_mag_tempco_mode mode)
{
    magTempcoMode = mode;
}

void TMAG5273::configConvAvgMode(tmag5273_conv_avg_mode mode)
{
    convAvgMode = mode;
}

void TMAG5273::configTempChEnabled(bool enabled)
{
    if (enabled) tempChEn = TMAG5273_TEMP_CH_ENABLED;
    else tempChEn = TMAG5273_TEMP_CH_DISABLED;
}

void TMAG5273::readRegister(uint8_t reg, uint8_t *data)
{
    i2c_dev->beginTransmission(currentDeviceAddress);
    i2c_dev->write(reg);
    i2c_dev->endTransmission(1);
    i2c_dev->requestFrom(currentDeviceAddress, 1);
    while (!i2c_dev->available()); 
    *data = i2c_dev->read();
}

void TMAG5273::readRegister(uint8_t reg, int16_t *data)
{
    i2c_dev->beginTransmission(currentDeviceAddress);
    i2c_dev->write(reg);
    i2c_dev->endTransmission(1);
    i2c_dev->requestFrom(currentDeviceAddress, 2);
    while (!i2c_dev->available()); 
    *data = (i2c_dev->read() << 8) | i2c_dev->read();
}

bool TMAG5273::writeRegister(uint8_t reg, uint8_t data)
{
    i2c_dev->beginTransmission(currentDeviceAddress);
    i2c_dev->write(reg);
    i2c_dev->write(data);
    return (bool) i2c_dev->endTransmission(1);
}

bool TMAG5273::writeRegisterGeneral(uint8_t reg, uint8_t data)
{
    i2c_dev->beginTransmission(0x00);
    i2c_dev->write(reg);
    i2c_dev->write(data);
    return (bool) i2c_dev->endTransmission(1);
}

void TMAG5273::switchSensor(uint8_t addr) 
{
    currentDeviceAddress = addr;
}

int TMAG5273::scanSensors(void) 
{

}

void TMAG5273::printDeviceTable(HardwareSerial* serial)
{

}

void TMAG5273::_init(void) 
{
    writeRegister(DEVICE_CONFIG_1, (REG_DEVICE_CONFIG_1_DEFAULT & 0x80) | magTempcoMode | convAvgMode | readMode); 
    writeRegister(DEVICE_CONFIG_2, (REG_DEVICE_CONFIG_2_DEFAULT & 0xEC) | lplnMode | operatingMode);
    writeRegister(SENSOR_CONFIG_1, REG_SENSOR_CONFIG_1_DEFAULT); 
    writeRegister(SENSOR_CONFIG_2, (REG_SENSOR_CONFIG_2_DEFAULT & 0xFC) | magRange); 
    writeRegister(X_THR_CONFIG, REG_X_THR_CONFIG_DEFAULT);
    writeRegister(Y_THR_CONFIG, REG_Y_THR_CONFIG_DEFAULT);
    writeRegister(Z_THR_CONFIG, REG_Z_THR_CONFIG_DEFAULT);
    writeRegister(T_CONFIG, (REG_T_CONFIG_DEFAULT & 0xFE) | tempChEn);
    writeRegister(INT_CONFIG_1, REG_INT_CONFIG_1_DEFAULT); 
    writeRegister(MAG_GAIN_CONFIG, REG_MAG_GAIN_CONFIG_DEFAULT);
    writeRegister(MAG_OFFSET_CONFIG_1, REG_MAG_OFFSET_CONFIG_1_DEFAULT);
    writeRegister(MAG_OFFSET_CONFIG_2, REG_MAG_OFFSET_CONFIG_2_DEFAULT);
}

void TMAG5273::initAll(void) 
{
    writeRegisterGeneral(DEVICE_CONFIG_1, (REG_DEVICE_CONFIG_1_DEFAULT & 0x80) | magTempcoMode | convAvgMode | readMode); // TODO support CRC
    writeRegisterGeneral(DEVICE_CONFIG_2, (REG_DEVICE_CONFIG_2_DEFAULT & 0xEC) | lplnMode | operatingMode);
    writeRegisterGeneral(SENSOR_CONFIG_1, REG_SENSOR_CONFIG_1_DEFAULT); // TODO MAG_CH_EN, SLEEPTIME setting
    writeRegisterGeneral(SENSOR_CONFIG_2, (REG_SENSOR_CONFIG_2_DEFAULT & 0xFC) | magRange); // TODO support ANGLE_EN and read
    writeRegisterGeneral(X_THR_CONFIG, REG_X_THR_CONFIG_DEFAULT);
    writeRegisterGeneral(Y_THR_CONFIG, REG_Y_THR_CONFIG_DEFAULT);
    writeRegisterGeneral(Z_THR_CONFIG, REG_Z_THR_CONFIG_DEFAULT);
    writeRegisterGeneral(T_CONFIG, (REG_T_CONFIG_DEFAULT & 0xFE) | tempChEn);
    writeRegisterGeneral(INT_CONFIG_1, REG_INT_CONFIG_1_DEFAULT); // INT pin disabled by default (connected to GND), TODO support INT pin enabled
    writeRegisterGeneral(MAG_GAIN_CONFIG, REG_MAG_GAIN_CONFIG_DEFAULT);
    writeRegisterGeneral(MAG_OFFSET_CONFIG_1, REG_MAG_OFFSET_CONFIG_1_DEFAULT);
    writeRegisterGeneral(MAG_OFFSET_CONFIG_2, REG_MAG_OFFSET_CONFIG_2_DEFAULT);
}

void TMAG5273::modifyI2CAddress(uint8_t new_addr)
{
    writeRegister(I2C_ADDRESS, (new_addr << 1) | 0x01);
    currentDeviceAddress = new_addr;
}

float TMAG5273::readTemperature() 
{
    // only used at READ_MODE_STANDARD
    int16_t TADCT;
    readRegister(T_MSB_RESULT, &TADCT);
    return TSENSET0 + (TADCT - TADCT0) / TADCRES;
}

uint8_t TMAG5273::readMagneticField(float* Bx, float* By, float* Bz, float* T) // Assume read all XYZ data
{
    // Standby mode seems can only work with READ_MODE_STANDARD without int pin where exists trigger bit
    // in READ_MODE_SENSOR16, not improve the efficiency if send trigger bit through I2C
    // currently no interrupt support, OPERATING_MODE_STANDBY works with READ_MODE_STANDARD, OPERATING_MODE_MEASURE works with READ_MODE_SENSOR16 for higher data throughput
    if (readMode == TMAG5273_READ_MODE_STANDARD) 
    {
        i2c_dev->beginTransmission(currentDeviceAddress);
        //if (operatingMode == TMAG5273_OPERATING_MODE_STANDBY) i2c_dev->write(X_MSB_RESULT | 0x80); 
        //else i2c_dev->write(X_MSB_RESULT);
        i2c_dev->write(X_MSB_RESULT | 0x80); // it seems ok to contain trigger bit at other modes
        i2c_dev->endTransmission(1);
    }
    // TODO support MODE_SENSOR8 1-byte read command for 8-bit data
    // TODO support MAG_CH_EN for other setup
    if (tempChEn == TMAG5273_TEMP_CH_ENABLED) i2c_dev->requestFrom(currentDeviceAddress, 9);
    else i2c_dev->requestFrom(currentDeviceAddress, 7);
    
    while (!i2c_dev->available()); 
    if (tempChEn == TMAG5273_TEMP_CH_ENABLED) {
        int16_t TADCT = (i2c_dev->read() << 8) | i2c_dev->read();
        *T = TSENSET0 + (TADCT - TADCT0) / TADCRES;
    }

    int16_t xData = (i2c_dev->read() << 8) | i2c_dev->read(); // X Mag MSB, LSB   2's complement auto translate
    int16_t yData = (i2c_dev->read() << 8) | i2c_dev->read(); // Y Mag MSB, LSB
    int16_t zData = (i2c_dev->read() << 8) | i2c_dev->read(); // Z Mag MSB, LSB
    uint8_t conv_status = i2c_dev->read();

    *Bx = magRangeValue * ((float) xData) / 32768.f;
    *By = magRangeValue * ((float) yData) / 32768.f;
    *Bz = magRangeValue * ((float) zData) / 32768.f;

    return conv_status;
}

uint8_t TMAG5273::readMagneticField(float* Bx, float* By, float* Bz)
{
    return readMagneticField(Bx, By, Bz, &_temp);
}
