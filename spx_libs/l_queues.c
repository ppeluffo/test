/*
 * l_queues.c
 *
 *  Created on: 2 jul. 2018
 *      Author: pablo
 */

#include "l_queues.h"

// ------------------------------------------------------------------------------------
// El FRTOS maneja colas de un tamano maximo de 255 bytes lo cual es poco
// para tareas de comunicaciones como en el caso del GPRS que el RX frame puede llegar
// a ser de 600 bytes.
// Por otro lado, para ahorrar memoria, es interesante poder acceder al buffer de las colas
// y que no sea una estructura protegida por el FRTOS.
// Creamos una estrucutra light de colas apta solo para nuestra aplicacion de GPRS.
// En las otras UARTs no tenemos problemas por lo cual usamos el sistema de colas provisto por
// el FRTOS.
// ------------------------------------------------------------------------------------
void l_QueueCreateStatic(uint16_t length,uint8_t *queueStorageBuffer, l_queue_handle_t *lQueue )
{
	lQueue->buff = queueStorageBuffer;
	lQueue->head = 0;
	lQueue->tail = 0;
	lQueue->uxMessageWaiting = 0;
	lQueue->length = length;

}
// ------------------------------------------------------------------------------------
void l_QueueReset( l_queue_handle_t *lQueue )
{

	taskENTER_CRITICAL();

	lQueue->head = 0;
	lQueue->tail = 0;
	lQueue->uxMessageWaiting = 0;
	memset(lQueue->buff,'\0', lQueue->length );

	taskEXIT_CRITICAL();

}
// ------------------------------------------------------------------------------------
bool l_QueueSend( l_queue_handle_t *lQueue, const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

int8_t ret = false;

	taskENTER_CRITICAL();
	// Si el buffer esta vacio ajusto los punteros
	if( lQueue->uxMessageWaiting == 0) {
		lQueue->head = lQueue->tail = 0;
	}

	if ( lQueue->uxMessageWaiting < lQueue->length ) {
		lQueue->buff[lQueue->head] = *cChar;
		++lQueue->uxMessageWaiting;
		// Avanzo en modo circular
		lQueue->head = ( lQueue->head  + 1 ) % ( lQueue->length );
		ret = true;
    }
	taskEXIT_CRITICAL();
	return(ret);

}
// ------------------------------------------------------------------------------------
bool l_QueueSendSendFromISR( l_queue_handle_t *lQueue, const char *cChar, TickType_t xTicksToWait )
{
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

int8_t ret = false;

	// Si el buffer esta vacio ajusto los punteros
	if( lQueue->uxMessageWaiting == 0) {
		lQueue->head = lQueue->tail = 0;
	}

	if ( lQueue->uxMessageWaiting < lQueue->length ) {
		lQueue->buff[lQueue->head] = *cChar;
		++lQueue->uxMessageWaiting;
		// Avanzo en modo circular
		lQueue->head = ( lQueue->head  + 1 ) % ( lQueue->length );
		ret = true;
	}

	return(ret);

}
// ------------------------------------------------------------------------------------
bool l_QueueReceive( l_queue_handle_t *lQueue, char *cChar, TickType_t xTicksToWait )
{

bool ret = false;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( lQueue->uxMessageWaiting == 0) {
		lQueue->head = lQueue->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = lQueue->buff[lQueue->tail];
	--lQueue->uxMessageWaiting;
	// Avanzo en modo circular
	lQueue->tail = ( lQueue->tail  + 1 ) % ( lQueue->length );
	ret = true;
	taskEXIT_CRITICAL();

	return(ret);
}
// ------------------------------------------------------------------------------------
bool l_QueueReceiveFromISR( l_queue_handle_t *lQueue, char *cChar, TickType_t xTicksToWait )
{

bool ret = false;

	//  Si el buffer esta vacio retorno.
	if( lQueue->uxMessageWaiting == 0) {
		lQueue->head = lQueue->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = lQueue->buff[lQueue->tail];
	--lQueue->uxMessageWaiting;
	// Avanzo en modo circular
	lQueue->tail = ( lQueue->tail  + 1 ) % ( lQueue->length );
	ret = true;

	return(ret);
}
/*------------------------------------------------------------------------------------*/
// UART ISR:
/*------------------------------------------------------------------------------------*/
