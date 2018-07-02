/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;
uint16_t i = 0;

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
	xprintf_P(pUSB,PSTR("\r\nstarting tkControl..\r\n\0"));

	for( ;; )
	{

		WDT_Reset();

		vTaskDelay( ( TickType_t)( 3000 / portTICK_PERIOD_MS ) );
		IO_set_LED_KA();
		vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
		IO_clr_LED_KA();
		xprintf_P(pUSB,PSTR("\tControl %d\r\n\0"),i++);

	}
}
//------------------------------------------------------------------------------------
