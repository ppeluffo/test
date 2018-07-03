/*
 * sp5K_i2c.h
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KDRIVERS_SP5K_I2C_H_
#define SRC_SP5KDRIVERS_SP5K_I2C_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/twi.h>

#include "FreeRTOS.h"
#include "task.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 	0
#define NACK 	1

#define I2C_SCL_BITPOS		1
#define I2C_SCL_PORT		PORTE

#define I2C_config_SCL()	( (&I2C_SCL_PORT)->DIR = (&I2C_SCL_PORT)->DIR | (1 << I2C_SCL_BITPOS ) )
#define I2C_set_SCL()		( (&I2C_SCL_PORT)->OUT = (&I2C_SCL_PORT)->OUT | (1 << I2C_SCL_BITPOS ) )
#define I2C_clr_SCL()		( (&I2C_SCL_PORT)->OUT = (&I2C_SCL_PORT)->OUT & ~(1 << I2C_SCL_BITPOS ) )

void drvI2C_init(void);
bool drvI2C_master_write ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes );
bool drvI2C_master_read  ( const uint8_t devAddress, const uint8_t devAddressLength, const uint16_t byteAddress, char *pvBuffer, size_t xBytes );

//#define DEBUG_I2C

#endif /* SRC_SP5KDRIVERS_SP5K_I2C_H_ */
