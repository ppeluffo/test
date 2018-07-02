/*
 * uarts_sp5K.c
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */

//#include "l_uarts.h"
#include "frtos_io.h"
#include "FreeRTOS.h"
//------------------------------------------------------------------------------------
void CMD_write( const void *pvBuffer, const size_t xBytes )
{
	// En el SP6K el USB y BT operan juntos como I/O de la tarea de comando
	// Para simplificar la escritura usamos esta funcion de modo que en el programa
	// no tenemos que escribir en ambos handles.

//uint16_t ticks_to_flush_queue;

	sFRTOS_write( pUSB, pvBuffer, xBytes );
	return;

}
//------------------------------------------------------------------------------------
void CMD_writeChar (unsigned char c)
{
	// Funcion intermedia necesaria por cmdline para escribir de a 1 caracter en consola
	// El tema es que el prototipo de funcion que requiere cmdlineSetOutputFunc no se ajusta
	// al de FreeRTOS_UART_write, por lo que defino esta funcion intermedia.

char cChar;

	cChar = c;
	CMD_write( &cChar, sizeof(char));
}
//------------------------------------------------------------------------------------
