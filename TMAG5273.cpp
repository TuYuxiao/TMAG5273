
#include "TMAG5273.h"


TMAG5273::TMAG5273(TwoWire *wire) {i2c_dev = wire;}

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
    switch (range) {
        case TMAG5273_MAG_RANGE_40MT:
        case TMAG5273_MAG_RANGE_133MT:
            magRangeMultiple = TMAG5273_MAG_RANGE_MULTIPLE_1x;
            break;
        case TMAG5273_MAG_RANGE_80MT:
        case TMAG5273_MAG_RANGE_266MT:
            magRangeMultiple = TMAG5273_MAG_RANGE_MULTIPLE_2x;
            break;
    }
    magRangeValue = (float) range;
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
    estiConversionTime = CONV_TIME_FROM_AVG_MODE(tempChEn, convAvgMode);
}

void TMAG5273::configTempChEnabled(bool enabled)
{
    if (enabled) tempChEn = TMAG5273_TEMP_CH_ENABLED;
    else tempChEn = TMAG5273_TEMP_CH_DISABLED;
    estiConversionTime = CONV_TIME_FROM_AVG_MODE(tempChEn, convAvgMode);
}

bool TMAG5273::readRegister(uint8_t reg, uint8_t *data)
{
    i2c_dev->beginTransmission(currentDeviceAddress);
    i2c_dev->write(reg);
    if (i2c_dev->endTransmission(1) != ESP_OK) return false;
    if (i2c_dev->requestFrom((int)currentDeviceAddress, 1) == 0) return false;
    while (!i2c_dev->available()); 
    *data = i2c_dev->read();
    return true;
}

bool TMAG5273::readRegister(uint8_t reg, int16_t *data)
{
    i2c_dev->beginTransmission(currentDeviceAddress);
    i2c_dev->write(reg);
    if (i2c_dev->endTransmission(1) != ESP_OK) return false;
    if (i2c_dev->requestFrom((int)currentDeviceAddress, 2) == 0) return false;
    while (!i2c_dev->available()); 
    *data = (i2c_dev->read() << 8) | i2c_dev->read();
    return true;
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

void TMAG5273::waitSensorArrayOff(uint32_t rc_delay)
{
    digitalWrite(2, HIGH);
    byte address, error;
    bool anyDev = true;
    while(anyDev) 
    {
        anyDev = false;
        for(address = 0x35; address < 0x65; address++)
        {
            i2c_dev->beginTransmission(address);
            error = i2c_dev->endTransmission();
            if (error == 0) 
            {
                anyDev = true;
                break;
            }
        }
        delay(1);
    }
    delay(rc_delay);
}

uint8_t TMAG5273::initSensorArray(uint32_t timeout)
{
    timeout = timeout * 1000;
    unsigned long start_time = micros();
    uint8_t error;
    arrayDevices = 0;
    while (1)
    {
        while((micros() - start_time) < timeout) 
        {
            i2c_dev->beginTransmission(default_addr);
            error = i2c_dev->endTransmission();
            if (error == 0)
            {
                switchSensor(default_addr);
                modifyI2CAddress(default_addr + 1 + arrayDevices);

                i2c_dev->beginTransmission(default_addr + 1 + arrayDevices);
                error = i2c_dev->endTransmission();
                if (error == 0) 
                {
                    arrayDevices++;
                    break;
                }
            }
            delay(1);
        }
        if ((micros() - start_time) < timeout) start_time = micros();
        else return arrayDevices;
    }
}

uint8_t TMAG5273::readSensorArray(float* data_ptr) 
{
    if (operatingMode == TMAG5273_OPERATING_MODE_STANDBY)
    {
        writeRegisterGeneral(DEVICE_ID | 0x80, 0x00); // trigger conversion after the register address decoding is completed, any readonly register is ok
        delayMicroseconds(estiConversionTime); // wait for conversion
    }
    uint8_t conv_status = 0x01;
    for(uint8_t addr = default_addr + 1; addr < default_addr + 1 + arrayDevices; addr++)
    {
        switchSensor(addr);
        conv_status &= _readMagneticField(data_ptr, data_ptr + 1, data_ptr + 2, &_temp);
        data_ptr += 3;
    }
    return conv_status;
}

void TMAG5273::printDeviceTable(HardwareSerial* serial)
{
    uint8_t error;
    serial->println();
    serial->print("Sensor num: ");
    serial->println(arrayDevices);
    for(uint8_t addr = default_addr + 1; addr < default_addr + 1 + arrayDevices; addr++)
    {
        i2c_dev->beginTransmission(addr);
        error = i2c_dev->endTransmission();
        if (error == 0) 
        {
            serial->print("0x");
            if (addr < 0x10) serial->print("0");
            serial->print(addr, HEX);
        }
        else serial->print("0x00");
        
        serial->print(" ");
    }
    serial->println("");
}

void TMAG5273::init(void) 
{
    writeRegister(DEVICE_CONFIG_1, (REG_DEVICE_CONFIG_1_DEFAULT & 0x80) | magTempcoMode | convAvgMode | readMode); 
    writeRegister(DEVICE_CONFIG_2, (REG_DEVICE_CONFIG_2_DEFAULT & 0xEC) | lplnMode | operatingMode);
    writeRegister(SENSOR_CONFIG_1, REG_SENSOR_CONFIG_1_DEFAULT); 
    writeRegister(SENSOR_CONFIG_2, (REG_SENSOR_CONFIG_2_DEFAULT & 0xFC) | magRangeMultiple); 
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
    writeRegisterGeneral(SENSOR_CONFIG_2, (REG_SENSOR_CONFIG_2_DEFAULT & 0xFC) | magRangeMultiple); // TODO support ANGLE_EN and read
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

int TMAG5273::readMagneticField(float* Bx, float* By, float* Bz, float* T) 
{
    if (operatingMode == TMAG5273_OPERATING_MODE_STANDBY)
    {
        writeRegister(DEVICE_ID | 0x80, 0x00); // trigger conversion after the register address decoding is completed
        delayMicroseconds(estiConversionTime); // wait for conversion 
    }

    return _readMagneticField(Bx, By, Bz, T);
}

int TMAG5273::_readMagneticField(float* Bx, float* By, float* Bz, float* T) // Assume read all XYZ data
{
    // Standby mode seems can only work with READ_MODE_STANDARD without int pin where exists trigger bit
    // in READ_MODE_SENSOR16, not improve the efficiency if send trigger bit through I2C
    // currently no interrupt support, OPERATING_MODE_STANDBY works with READ_MODE_STANDARD, OPERATING_MODE_MEASURE works with READ_MODE_SENSOR16 for higher data throughput
    if (readMode == TMAG5273_READ_MODE_STANDARD) 
    {
        i2c_dev->beginTransmission(currentDeviceAddress);
        if (tempChEn == TMAG5273_TEMP_CH_ENABLED) i2c_dev->write(T_MSB_RESULT);
        else i2c_dev->write(X_MSB_RESULT); // it seems ok to contain trigger bit at other modes
        if (i2c_dev->endTransmission(1) != ESP_OK) {
            return -1;
        }
    }

    byte data_buffer[9];
    int read_bytes;
    if (tempChEn == TMAG5273_TEMP_CH_ENABLED) read_bytes = 9; else read_bytes = 7;

    // TODO support MODE_SENSOR8 1-byte read command for 8-bit data
    // TODO support MAG_CH_EN for other setup
    if (i2c_dev->requestFrom((int)currentDeviceAddress, read_bytes) == 0) {
        log_e("Request data error");
        return -1;
    }
    
    unsigned long start_time = micros();
    while (!i2c_dev->available()) {
        if ((micros() - start_time) > 1e5) { //timeout 100 ms
            log_e("Wait read timeout");
            return -1;
        }
    } 
    
    for (int i=9-read_bytes; i<9; i++) {
        data_buffer[i] = i2c_dev->read();
        if (data_buffer[i] < 0) {
            log_e("Read data error");
            return -1;
        }
    }

    if (tempChEn == TMAG5273_TEMP_CH_ENABLED) {
        int16_t TADCT = (data_buffer[0] << 8) | data_buffer[1];
        *T = TSENSET0 + (TADCT - TADCT0) / TADCRES;
    }

    int16_t xData = (data_buffer[2] << 8) | data_buffer[3]; // X Mag MSB, LSB   2's complement auto translate
    int16_t yData = (data_buffer[4] << 8) | data_buffer[5]; // Y Mag MSB, LSB
    int16_t zData = (data_buffer[6] << 8) | data_buffer[7]; // Z Mag MSB, LSB
    uint8_t conv_status = data_buffer[8];

    *Bx = magRangeValue * ((float) xData) / 32768.f;
    *By = magRangeValue * ((float) yData) / 32768.f;
    *Bz = magRangeValue * ((float) zData) / 32768.f;

    return conv_status;
}

int TMAG5273::readMagneticField(float* Bx, float* By, float* Bz)
{
    return readMagneticField(Bx, By, Bz, &_temp);
}

void TMAG5273::setDefaultAddr(uint8_t addr)
{
    default_addr = addr;
    arrayDevices = 0;
}
