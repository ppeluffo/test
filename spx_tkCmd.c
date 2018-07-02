/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */
#include <spx.h>

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

( void ) pvParameters;
uint16_t i = 0;

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
	xprintf_P(pUSB,PSTR("\r\nstarting tkCmd..\r\n\0"));

	for( ;; )
	{

		vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
		IO_set_LED_COMMS();
		vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
		IO_clr_LED_COMMS();
		xprintf_P(pUSB,PSTR("Cmd %d\r\n\0"),i++);
	}
}
//------------------------------------------------------------------------------------
