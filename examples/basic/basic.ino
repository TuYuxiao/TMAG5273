#include <Arduino.h>
#include <Wire.h>
#include "TMAG5273.h"

#define TMAG_SDA 25
#define TMAG_SCL 26

TMAG5273 tmag5273(&Wire);

void setup() 
{
  Wire.begin(TMAG_SDA , TMAG_SCL, 400000);
  Serial.begin(115200);
  while (!Serial); 

  tmag5273.modifyI2CAddress(0x36); // change I2C address
  
  tmag5273.configOperatingMode(TMAG5273_OPERATING_MODE_STANDBY);
  tmag5273.configReadMode(TMAG5273_READ_MODE_STANDARD);
  tmag5273.configMagRange(TMAG5273_MAG_RANGE_80MT);
  tmag5273.configLplnMode(TMAG5273_LOW_NOISE);
  tmag5273.configMagTempcoMode(TMAG5273_MAG_TEMPCO_NdBFe);
  tmag5273.configConvAvgMode(TMAG5273_CONV_AVG_32X);
  tmag5273.configTempChEnabled(true);
  tmag5273.init();
}



void loop() 
{
  delay(1000);
  float Bx, By, Bz, T;
  uint8_t res;
  
  res = tmag5273.readMagneticField(&Bx, &By, &Bz, &T);

  Serial.println("The temperature data is: " + String(T));

  Serial.println("X Axis Magnetic Field in mT: " + String(Bx));
  Serial.println("Y Axis Magnetic Field in mT: " + String(By));
  Serial.println("Z Axis Magnetic Field in mT: " + String(Bz));
  Serial.println(res & 0x1); // conv complete status bit
}
