/*
 * frtos_io.c
 *
 *  Created on: 27 jun. 2018
 *      Author: pablo
 */


#include "frtos_io.h"

//----------------------------------------------------------------------------
// FUNCIONES GENERALES DE FRTOS-IO
//----------------------------------------------------------------------------
int sFRTOS_open( int fd, const uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada.
	// Es la que invoca la aplicacion.

	int8_t retS = -1;

	switch(fd) {
	case pUSB:
		spd_USB.uart_fd = pUSB;
		retS = sFRTOS_UART_open( &spd_USB, flags);
		break;
	}

	return(retS);
}
//----------------------------------------------------------------------------
int sFRTOS_ioctl( int fd, uint32_t ulRequest, void *pvValue )
{
	int8_t retS = -1;

	switch(fd) {
	case pUSB:
		retS = sFRTOS_UART_ioctl( &spd_USB, ulRequest, pvValue);
		break;
	}

	return(retS);

}
//----------------------------------------------------------------------------
int sFRTOS_write( int fd, const char *pvBuffer, const size_t xBytes )
{

	int8_t retS = -1;

	switch(fd) {
	case pUSB:
		retS = sFRTOS_UART_write( &spd_USB, pvBuffer, xBytes );
		break;
	}

	return(retS);
}
//----------------------------------------------------------------------------
int sFRTOS_read( int fd, char *pvBuffer, const size_t xBytes )
{
	int8_t retS = -1;

	switch(fd) {
	case pUSB:
		retS = sFRTOS_UART_read( &spd_USB, pvBuffer, xBytes );
		break;
	}

	return(retS);
}
//----------------------------------------------------------------------------
// FUNCIONES PARTICULARES DE FRTOS-IO PARA UARTs
//----------------------------------------------------------------------------
int sFRTOS_UART_open( t_serial_port_device *spd, const uint32_t flags)
{
	// Funcion privada especializada en abrir un puerto serial ( uart )
	// No se invoca directamente !!!
	// Las queue no pueden ser mayores a 256 bytes.
	//
	int8_t retS = -1;

	switch(spd->uart_fd) {
	case pUSB:
		// Abro el uart asociado al USB
		spd->xBlockTime = (10 / portTICK_RATE_MS );
		spd->xBusSemaphore = xSemaphoreCreateMutex();
		spd->uart.rxQueue_size = UART_USB_RXBUFFER_SIZE;
		spd->uart.txQueue_size = UART_USB_TXBUFFER_SIZE;
		spd->uart.rxQueue = xQueueCreate( spd->uart.rxQueue_size, sizeof( char ) );
		spd->uart.txQueue = xQueueCreate( spd->uart.txQueue_size, sizeof( char ) );
		drvUART_open(spd->uart_fd);
		break;
	}
	return(retS);
}
//----------------------------------------------------------------------------
int sFRTOS_UART_ioctl ( t_serial_port_device *spd, uint32_t ulRequest, void *pvValue )
{

portBASE_TYPE xReturn = pdPASS;

	switch( ulRequest ) {
	case ioctlOBTAIN_BUS_SEMPH:
		// Espero el semaforo en forma persistente.
		while ( xSemaphoreTake( spd->xBusSemaphore, ( TickType_t ) 1 ) != pdTRUE )
			taskYIELD();
		break;
	case ioctlRELEASE_BUS_SEMPH:
		xSemaphoreGive( spd->xBusSemaphore );
		break;
	case ioctlSET_TIMEOUT:
		spd->xBlockTime = *((uint8_t *)pvValue);
		break;
	case ioctl_UART_CLEAR_RX_QUEUE:
		xQueueReset(spd->uart.rxQueue);
		break;
	case ioctl_UART_CLEAR_TX_QUEUE:
		xQueueReset(spd->uart.rxQueue);
		break;
	default :
		xReturn = pdFAIL;
		break;
	}

	return xReturn;


}
//----------------------------------------------------------------------------
int sFRTOS_UART_write( t_serial_port_device *spd, const char *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision.
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
size_t bytes2tx;
size_t wBytes = 0;

	bytes2tx = xBytes;

	// Espero el semaforo en forma persistente.
	while ( xSemaphoreTake( spd->xBusSemaphore, ( TickType_t ) 1 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Trasmito.
	// La cola ya esta vacia al tomar el semaforo

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		xQueueSend( spd->uart.txQueue, &cChar, ( TickType_t ) 10  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo
		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if ( uxQueueMessagesWaiting( spd->uart.txQueue ) > (int)( 0.8 * spd->uart.txQueue_size )) {
			// Habilito a trasmitir para que comienze a vaciarse
			drvUART_InterruptOn(spd->uart_fd);
			// Y espero que se haga mas lugar.
			while ( uxQueueMessagesWaiting( spd->uart.txQueue ) > (int)(0.2 * spd->uart.txQueue_size ))
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	drvUART_InterruptOn(spd->uart_fd);

	// Espero que trasmita todo
	while ( uxQueueMessagesWaiting( spd->uart.txQueue ) > (int)(0.2 * spd->uart.txQueue_size ))
		vTaskDelay( ( TickType_t)( 1 ) );

	xSemaphoreGive( spd->xBusSemaphore );

	//return xBytes;	// Puse todos los caracteres en la cola.
	return (wBytes);

}
//----------------------------------------------------------------------------
int sFRTOS_UART_read( t_serial_port_device *spd, char *pvBuffer, const size_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

size_t xBytesReceived = 0U;
portTickType xTicksToWait;
xTimeOutType xTimeOut;

	xTicksToWait = spd->xBlockTime;
	xTicksToWait = 10;
	vTaskSetTimeOutState( &xTimeOut );

	/* Are there any more bytes to be received? */
	while( xBytesReceived < xBytes )
	{
		/* Receive the next character. */
		if( xQueueReceive( spd->uart.rxQueue, &((char *)pvBuffer)[ xBytesReceived ], xTicksToWait ) == pdPASS ) {
			xBytesReceived++;
		}

		// Espero xTicksToWait antes de volver a chequear
		vTaskDelay( ( TickType_t)( xTicksToWait ) );

		/* Time out has expired ? */
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return xBytesReceived;

}
//----------------------------------------------------------------------------


