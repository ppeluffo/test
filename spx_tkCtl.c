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

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
	cmd_xprintf_P(pUSB,PSTR("\r\nstarting tkControl..\r\n\0"));

	for( ;; )
	{

		WDT_Reset();

		vTaskDelay( ( TickType_t)( 3000 / portTICK_PERIOD_MS ) );
		IO_set_LED_KA();
		vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
		IO_clr_LED_KA();

	}
}
//------------------------------------------------------------------------------------
