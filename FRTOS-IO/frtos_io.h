/*
 * frtos_io.h
 *
 *  Created on: 27 jun. 2018
 *      Author: pablo
 */

#ifndef FRTOS_IO_FRTOS_IO_H_
#define FRTOS_IO_FRTOS_IO_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <avr/interrupt.h>
#include "drv_uart_spx.h"

typedef enum {

	pUSB = 0,

} t_fd;

typedef struct {

	QueueHandle_t txQueue;
	uint8_t txQueue_size;
	QueueHandle_t rxQueue;
	uint8_t rxQueue_size;
} t_uart;

typedef struct {

	t_fd uart_fd;
	SemaphoreHandle_t xBusSemaphore;		//
	uint8_t xBlockTime;						// ticks to block in read operations. Set by ioctl
	t_uart uart;

} t_serial_port_device;

// Creo las estructuras de los dispositivos

t_serial_port_device spd_USB;

// Las queue no pueden ser mayores a 256 bytes.
#define  UART_USB_RXBUFFER_SIZE ( ( uint8_t ) ( 128 ))
#define  UART_USB_TXBUFFER_SIZE ( ( uint8_t ) ( 128 ))

#define ioctlOBTAIN_BUS_SEMPH		1
#define ioctlRELEASE_BUS_SEMPH		2
#define ioctlSET_TIMEOUT			3
#define ioctl_UART_CLEAR_RX_QUEUE	4
#define ioctl_UART_CLEAR_TX_QUEUE	5


int sFRTOS_open( int fd, const uint32_t flags);
int sFRTOS_ioctl( int fd, uint32_t ulRequest, void *pvValue );
int sFRTOS_write( int fd, const char *pvBuffer, const size_t xBytes );
int sFRTOS_read( int fd, char *pvBuffer, const size_t xBytes );

int sFRTOS_UART_open( t_serial_port_device *spd, const uint32_t flags);
int sFRTOS_UART_write( t_serial_port_device *spd, const char *pvBuffer, const size_t xBytes );
int sFRTOS_UART_ioctl ( t_serial_port_device *spd, uint32_t ulRequest, void *pvValue );
int sFRTOS_UART_read( t_serial_port_device *spd, char *pvBuffer, const size_t xBytes );


#endif /* FRTOS_IO_FRTOS_IO_H_ */
