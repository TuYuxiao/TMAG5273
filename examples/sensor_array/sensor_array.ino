#include <Arduino.h>
#include <Wire.h>
#include "TMAG5273.h"

#define TMAG_SDA 25
#define TMAG_SCL 26
#define TMAG_EN 2

TMAG5273 tmag5273(&Wire);

float* sensor_data;
uint8_t arrayDevices;

void setup() 
{
  pinMode(TMAG_EN, OUTPUT);
  Wire.begin(TMAG_SDA , TMAG_SCL, 400000); // up to 1MHz
  Serial.begin(115200);
  while (!Serial); 
  

  digitalWrite(TMAG_EN, HIGH);
  tmag5273.waitSensorArrayOff();
  // wait unitl all devices power off ...
  
  digitalWrite(TMAG_EN, LOW);
  arrayDevices = tmag5273.initSensorArray();
  sensor_data = (float *) malloc(arrayDevices * sizeof(float));
  Serial.println(arrayDevices);
  
  tmag5273.printDeviceTable(&Serial);

  tmag5273.configOperatingMode(TMAG5273_OPERATING_MODE_MEASURE);
  tmag5273.configReadMode(TMAG5273_READ_MODE_SENSOR16);
  tmag5273.configMagRange(TMAG5273_MAG_RANGE_80MT);
  tmag5273.configLplnMode(TMAG5273_LOW_NOISE);
  tmag5273.configMagTempcoMode(TMAG5273_MAG_TEMPCO_NdBFe);
  tmag5273.configConvAvgMode(TMAG5273_CONV_AVG_32X);
  tmag5273.configTempChEnabled(false);
  tmag5273.initAll(); // config all sensors by general call
}

void loop() 
{
  delay(1000);

  if (tmag5273.readSensorArray(sensor_data) == 0) {
    Serial.println("Conv not complete!");
    while(1);
  }
  Serial.println(micros());
  for (int i=0;i<arrayDevices;i++) {
    Serial.print(*(sensor_data+i*3));Serial.print(" mT  "); // X axis
    Serial.print(*(sensor_data+i*3+1));Serial.print(" mT  "); // Y axis
    Serial.print(*(sensor_data+i*3+2));Serial.println(" mT  "); // Z axis
  }
  Serial.println();Serial.println();Serial.println();
}
