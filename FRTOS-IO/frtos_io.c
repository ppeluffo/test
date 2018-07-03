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
	// Retorna -1 en error o un nro positivo ( fd )

int8_t retS = -1;

	switch(fd) {
	case pUSB:
		retS = sFRTOS_UART_USB_open( &spd_USB, flags );
		break;
	case pI2C:
		retS = sFRTOS_I2C_open( &spd_I2C, flags );
		break;
	case pNVM:
		retS = sFRTOS_NVM_open( &spd_NVM, flags );
		break;
	case pBT:
		retS = sFRTOS_UART_BT_open( &spd_BT, flags );
		break;
	default:
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
	case pI2C:
		retS = sFRTOS_I2C_ioctl( &spd_I2C, ulRequest, pvValue);
		break;
	case pNVM:
		retS = sFRTOS_NVM_ioctl( &spd_NVM, ulRequest, pvValue);
		break;
	case pBT:
		retS = sFRTOS_UART_ioctl( &spd_BT, ulRequest, pvValue);
		break;
	default:
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
		retS = sFRTOS_UART_queue_write( &spd_USB, pvBuffer, xBytes );
		break;
	case pI2C:
		retS = sFRTOS_I2C_write( &spd_I2C, pvBuffer, xBytes );
		break;
	case pNVM:
		retS = sFRTOS_NVM_write( &spd_NVM, pvBuffer, xBytes );
		break;
	case pBT:
		retS = sFRTOS_UART_queue_write( &spd_BT, pvBuffer, xBytes );
		break;
	default:
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
		retS = sFRTOS_UART_queue_read( &spd_USB, pvBuffer, xBytes );
		break;
	case pI2C:
		retS = sFRTOS_I2C_read( &spd_I2C, pvBuffer, xBytes );
		break;
	case pNVM:
		retS = sFRTOS_NVM_read( &spd_NVM, pvBuffer, xBytes );
		break;
	case pBT:
		retS = sFRTOS_UART_queue_read( &spd_BT, pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(retS);
}
//----------------------------------------------------------------------------
// FUNCIONES PARTICULARES DE FRTOS-IO PARA UARTs
//----------------------------------------------------------------------------
int sFRTOS_UART_USB_open( t_serial_port_device *spd, const uint32_t flags)
{
	// Funcion privada especializada en abrir un puerto serial ( uart )
	// No se invoca directamente !!!
	// Las queue no pueden ser mayores a 256 bytes.
	// Retorna el fd ( valor > 0 ).

	spd->uart_fd = pUSB;
	spd->xBlockTime = (10 / portTICK_RATE_MS );
	spd->xBusSemaphore = xSemaphoreCreateMutex();
	spd->uart.rxQueue_size = UART_USB_RXBUFFER_SIZE;
	spd->uart.txQueue_size = UART_USB_TXBUFFER_SIZE;
	spd->uart.rxQueue = xQueueCreate( spd->uart.rxQueue_size, sizeof( char ) );
	spd->uart.txQueue = xQueueCreate( spd->uart.txQueue_size, sizeof( char ) );
	drvUART_USB_open( flags );

	return(pUSB);
}
//----------------------------------------------------------------------------
int sFRTOS_UART_BT_open( t_serial_port_device *spd, const uint32_t flags)
{
	// Funcion privada especializada en abrir un puerto serial ( uart )
	// No se invoca directamente !!!
	// Las queue no pueden ser mayores a 256 bytes.
	// Retorna el fd ( valor > 0 ).

	spd->uart_fd = pBT;
	spd->xBlockTime = (10 / portTICK_RATE_MS );
	spd->xBusSemaphore = xSemaphoreCreateMutex();
	spd->uart.rxQueue_size = UART_BT_RXBUFFER_SIZE;
	spd->uart.txQueue_size = UART_BT_TXBUFFER_SIZE;
	spd->uart.rxQueue = xQueueCreate( spd->uart.rxQueue_size, sizeof( char ) );
	spd->uart.txQueue = xQueueCreate( spd->uart.txQueue_size, sizeof( char ) );
	drvUART_BT_open( flags );

	return(pBT);
}
//----------------------------------------------------------------------------
int sFRTOS_UART_ioctl ( t_serial_port_device *spd, uint32_t ulRequest, void *pvValue )
{

int xReturn = 1;

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
		xQueueReset(spd->uart.txQueue);
		break;
	default :
		xReturn = -1;
		break;
	}

	return (xReturn);

}
//----------------------------------------------------------------------------
int sFRTOS_UART_queue_write( t_serial_port_device *spd, const char *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision.
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
int bytes2tx;
int wBytes = 0;

	bytes2tx = xBytes;

	// Espero el semaforo en forma persistente.
	while ( xSemaphoreTake( spd->xBusSemaphore, ( TickType_t ) 5 ) != pdTRUE )
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
int sFRTOS_UART_queue_read( t_serial_port_device *spd, char *pvBuffer, const size_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int xBytesReceived = 0U;
portTickType xTicksToWait;
xTimeOutType xTimeOut;

	xTicksToWait = spd->xBlockTime;
	xTicksToWait = 2;
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{
		// Receive the next character
		if( xQueueReceive( spd->uart.rxQueue, &((char *)pvBuffer)[ xBytesReceived ], xTicksToWait ) == pdTRUE ) {
			xBytesReceived++;
		}
//		else {
			// Espero xTicksToWait antes de volver a chequear
//			vTaskDelay( ( TickType_t)( xTicksToWait ) );
//		}

		// Time out has expired ?
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return xBytesReceived;

}
//----------------------------------------------------------------------------
// FUNCIONES PARTICULARES DE FRTOS-IO PARA I2C
//----------------------------------------------------------------------------
int sFRTOS_I2C_open( t_i2c_device *spd, const uint32_t flags)
{
	// Funcion privada especializada en abrir un puerto serial ( uart )
	// No se invoca directamente !!!
	// Las queue no pueden ser mayores a 256 bytes.
	// Retorna el fd ( valor > 0 ).

	spd->i2c_fd = pI2C;
	spd->xBlockTime = (10 / portTICK_RATE_MS );
	spd->xBusSemaphore = xSemaphoreCreateMutex();
	drvI2C_init();

	return(pI2C);
}
//----------------------------------------------------------------------------
int sFRTOS_I2C_ioctl ( t_i2c_device *spd, uint32_t ulRequest, void *pvValue )
{

int xReturn = 1;
uint16_t *p;

	p = pvValue;

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
	case ioctl_I2C_SET_DEVADDRESS:
		spd->devAddress = (int8_t)(*p);
		break;
	case ioctl_I2C_SET_BYTEADDRESS:
		spd->byteAddress = (uint16_t)(*p);
		break;
	case ioctl_I2C_SET_BYTEADDRESSLENGTH:
		spd->byteAddressLength = (int8_t)(*p);
		break;
	default :
		xReturn = -1;
		break;
	}

	return (xReturn);

}
//----------------------------------------------------------------------------
int sFRTOS_I2C_write( t_i2c_device *spd, const void *pvBuffer, const size_t xBytes )
{
	// Esta operacion requiere que antes por medio de IOCTL se halla definido el
	// devaddress, byteaddress, length

int xReturn = -1;

#ifdef DEBUG_I2C
		FRTOS_snprintf_P( stdout_buff,CHAR128,PSTR("FRTOS_I2C_WR: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"),pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress, xBytes);
		CMD_write( stdout_buff, sizeof(stdout_buff) );
#endif

	if ( drvI2C_master_write(spd->devAddress, spd->byteAddressLength, spd->byteAddress, (char *)pvBuffer, xBytes) == true ) {
		xReturn = xBytes;
	}

//	spd->i2c_errno = I2C_EXIT_CODE;

	return(xReturn);
}
//----------------------------------------------------------------------------
int sFRTOS_I2C_read( t_i2c_device *spd, void * const pvBuffer, const size_t xBytes )
{
	// Esta operacion requiere que antes por medio de IOCTL se halla definido el
	// devaddress, byteaddress, length

int xReturn = -1;


#ifdef DEBUG_I2C
		FRTOS_snprintf_P( stdout_buff,CHAR128,PSTR("FRTOS_I2C_RD: devAddr:0x%02x,addrLen:0x%02x,byteAddr:0x%02x,xbytes: 0x%02x\r\n\0"),pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress, xBytes);
		CMD_write( stdout_buff, sizeof(stdout_buff) );
#endif

	if ( drvI2C_master_read(spd->devAddress, spd->byteAddressLength, spd->byteAddress, (char *)pvBuffer, xBytes) == true ) {
		xReturn = xBytes;
	}

//	spd->i2c_errno = I2C_EXIT_CODE;

	return(xReturn);

}
//----------------------------------------------------------------------------
// FUNCIONES PARTICULARES DE FRTOS-IO PARA NVM
//----------------------------------------------------------------------------
int sFRTOS_NVM_open( t_nvmc_device *spd, const uint32_t flags)
{
	// Retorna el fd ( valor > 0 ).

	spd->i2c_fd = pNVM;
	spd->xBlockTime = (10 / portTICK_RATE_MS );
	spd->xBusSemaphore = xSemaphoreCreateMutex();

	return(pNVM);
}
//----------------------------------------------------------------------------
int sFRTOS_NVM_ioctl ( t_nvmc_device *spd, uint32_t ulRequest, void *pvValue )
{

int xReturn = 1;
uint16_t *p;

	p = pvValue;

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
	case ioctl_NVM_SET_BYTEADDRESS:
		spd->byteAddress = (uint16_t)(*p);
		break;
	case ioctl_NVM_ERASE_ALL:
		drvNVM_EEPROM_EraseAll();
		break;
	case ioctl_NVM_READ_SIGNATURE:
		spd->signatureVal = drvNVM_ReadSignatureByte( spd->byteAddress);
		xReturn = spd->signatureVal;
		break;
	default :
		xReturn = -1;
		break;
	}

	return (xReturn);

}
//----------------------------------------------------------------------------
int sFRTOS_NVM_write( t_nvmc_device *spd, const void *pvBuffer, const size_t xBytes )
{
	// Esta operacion requiere que antes por medio de IOCTL se halla definido el
	// byteaddress que es la direccion a partir de donde se va a escribir

int xReturn = -1;

	drvNVM_EEPROM_write_buffer(spd->byteAddress, pvBuffer, xBytes);
	return(xReturn);
}
//------------------------------------------------------------------------------------
int sFRTOS_NVM_read( t_nvmc_device *spd, void * const pvBuffer, const size_t xBytes )
{
	// Esta operacion requiere que antes por medio de IOCTL se halla definido el
	// byteaddress que es la direccion a partir de donde se va a leer

int xReturn = 0;

	drvNVM_EEPROM_read_buffer(spd->byteAddress, pvBuffer, xBytes);
	return(xReturn);

}
//------------------------------------------------------------------------------------


