/*
 * Eeprom.h
 *
 *  Created on: Oct 27, 2022
 *      Author: Charlie
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#define _EEPROM_IC2_ADDR 		0x50		//the I2C bus address for the EEPROM
#define EEPROM_CONFIG_ADDRESS 	30
#define _EEPROMUPDATE_MILLIS 	30000

class Eeprom {
public:
	Eeprom();
	void write(uint16_t addr,uint8_t data);
	byte read(uint16_t addr);
	void readBuffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length );
	void saveDeviceConfig(byte b[], uint16_t addr, uint16_t size);
	void loadDeviceConfig(byte b[], uint16_t addr, uint16_t size);
private:
};



#endif /* EEPROM_H_ */
