/*
 * Eeprom.cpp
 *
 *  Created on: Oct 27, 2022
 *      Author: Charlie
 */

#include <Wire.h>
#include "Arduino.h"
#include "Eeprom.h"

Eeprom::Eeprom() {

}

void Eeprom::readBuffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
       if (Wire.available())
         buffer[c] = Wire.read();
    Wire.endTransmission();
}

void Eeprom::write(uint16_t addr,uint8_t data) {
  //Serial.print("EEPROM_write");
  Wire.beginTransmission(_EEPROM_IC2_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();    // stop transmitting
}

byte Eeprom::read(uint16_t addr) {
  byte data;
  Wire.beginTransmission(_EEPROM_IC2_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(_EEPROM_IC2_ADDR, 1);

  if (Wire.available()) {
    data = Wire.read();
  }
  Wire.endTransmission();

  return data;
}

void Eeprom::saveDeviceConfig(byte b[], uint16_t addr, uint16_t size) {
  for(uint16_t  i=0; i<size; i++) {
	// 10-27-22 EEPROM_write(EEPROM_CONFIG_ADDRESS + i, b[i]);
    write(EEPROM_CONFIG_ADDRESS + i, b[i]);
	Serial.print(i);
    //write(addr + i, b[i]);
    delay(10);
  }
}

void Eeprom::loadDeviceConfig(byte b[], uint16_t addr,uint16_t size) {
  for(uint16_t i=0; i<size; i++) {
    // 10-27-22 b[i] = EEPROM_read(EEPROM_CONFIG_ADDRESS + i);
    //b[i] = read(EEPROM_CONFIG_ADDRESS + i);
    b[i] = read(addr + i);
  }
}





