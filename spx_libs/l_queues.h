/*
 * l_queues.h
 *
 *  Created on: 2 jul. 2018
 *      Author: pablo
 */

#ifndef SPX_LIBS_L_QUEUES_H_
#define SPX_LIBS_L_QUEUES_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
	uint8_t *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t uxMessageWaiting;
	uint16_t length;
} l_queue_handle_t;

//------------------------------------------------------------------------------------

void l_QueueCreateStatic(uint16_t length,uint8_t *queueStorageBuffer, l_queue_handle_t *lQueue );
void l_QueueReset( l_queue_handle_t *lQueue );
bool l_QueueSend( l_queue_handle_t *lQueue, const char *cChar, TickType_t xTicksToWait );
bool l_QueueSendSendFromISR( l_queue_handle_t *lQueue, const char *cChar, TickType_t xTicksToWait );
bool l_QueueReceive( l_queue_handle_t *lQueue, char *cChar, TickType_t xTicksToWait );

#endif /* SPX_LIBS_L_QUEUES_H_ */
