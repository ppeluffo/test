/*
 * drv_usb_spx.h
 *
 *  Created on: 28 jun. 2018
 *      Author: pablo
 */

#ifndef DRIVERS_DRV_UART_SPX_H_
#define DRIVERS_DRV_UART_SPX_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <avr/interrupt.h>
#include "frtos_io.h"

void drvUART_open( const int UARTx );
void drvUART_InterruptOn( const int UARTx );
void drvUART_InterruptOff( const int UARTx );

#endif /* DRIVERS_DRV_UART_SPX_H_ */
