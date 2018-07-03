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

void drvNVM_EEPROM_write_buffer(uint16_t address, const void *buf, uint16_t len);
void drvNVM_EEPROM_write_byte(uint16_t address, uint8_t value);
void drvNVM_EEPROM_read_buffer(uint16_t address, char *buf, uint16_t len);
uint8_t drvNVM_EEPROM_ReadByte( uint16_t address );
void drvNVM_EEPROM_EraseAll( void );
uint8_t drvNVM_ReadSignatureByte(uint16_t Address);

#endif /* DRIVERS_DRV_NVM_SPX_H_ */
