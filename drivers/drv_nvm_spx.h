/*
 * drv_nvm_spx.h
 *
 *  Created on: 3 jul. 2018
 *      Author: pablo
 */

#ifndef DRIVERS_DRV_NVM_SPX_H_
#define DRIVERS_DRV_NVM_SPX_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#define NVM_EEPROM_SIZE	2048

struct nvm_device_id {
	union {
		struct {
			uint8_t devid0;
			uint8_t devid1;
			uint8_t devid2;
		};
		uint8_t byte[3];
	};
};

/**
 * \brief Structure containing the device serial
 *
 * This structure can be used to store the serial number of a device.
 */
struct nvm_device_serial {
	union {
		struct {
			uint8_t lotnum0;
			uint8_t lotnum1;
			uint8_t lotnum2;
			uint8_t lotnum3;
			uint8_t lotnum4;
			uint8_t lotnum5;
			uint8_t wafnum;
			uint8_t coordx0;
			uint8_t coordx1;
			uint8_t coordy0;
			uint8_t coordy1;
		};
		uint8_t byte[11];
	};
};

enum {
	LOTNUM0=8,  // Lot Number Byte 0, ASCII
	LOTNUM1,    // Lot Number Byte 1, ASCII
	LOTNUM2,    // Lot Number Byte 2, ASCII
	LOTNUM3,    // Lot Number Byte 3, ASCII
	LOTNUM4,    // Lot Number Byte 4, ASCII
	LOTNUM5,    // Lot Number Byte 5, ASCII
	WAFNUM =16, // Wafer Number
	COORDX0=18, // Wafer Coordinate X Byte 0
	COORDX1,    // Wafer Coordinate X Byte 1
	COORDY0,    // Wafer Coordinate Y Byte 0
	COORDY1,    // Wafer Coordinate Y Byte 1
};

#define nvm_get_production_signature_row_offset(regname) offsetof(NVM_PROD_SIGNATURES_t, regname)

void drvNVM_EEPROM_write_buffer(uint16_t address, const void *buf, uint16_t len);
void drvNVM_EEPROM_write_byte(uint16_t address, uint8_t value);
void drvNVM_EEPROM_read_buffer(uint16_t address, char *buf, uint16_t len);
uint8_t drvNVM_EEPROM_ReadByte( uint16_t address );
void drvNVM_EEPROM_EraseAll( void );
uint8_t drvNVM_ReadSignatureByte(uint16_t Address);

void nvm_read_device_id(struct nvm_device_id *storage);

#endif /* DRIVERS_DRV_NVM_SPX_H_ */
