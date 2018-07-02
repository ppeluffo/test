/*
 * drv_uart_spx.c
 *
 *  Created on: 28 jun. 2018
 *      Author: pablo
 */


#include "drv_uart_spx.h"

//----------------------------------------------------------------------------
void drvUART_open( const int UARTx )
{

	// La inicializacion de las UART se hace como:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

	portENTER_CRITICAL();

	//
	switch ( UARTx ) {
	case pUSB:

		// Corresponde a PORTD
		PORTD.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
		PORTD.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */
		/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
		USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

#if F_CPU == 32000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 32Mhz
		 * BSEL = 2094
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,01%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 2094;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
#endif

#if F_CPU == 8000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 983;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
		USARTD0.CTRLB |= USART_CLK2X_bm;
#endif

#if F_CPU == 2000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		USARTD0.BAUDCTRLA = (uint8_t) 11;
		USARTD0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif

		// Habilito la TX y RX
		USARTD0.CTRLB |= USART_RXEN_bm;
		USARTD0.CTRLB |= USART_TXEN_bm;

		// Habilito la interrupcion de Recepcion ( low level )
		// low level, RXint enabled
		USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
		USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
		//USARTD0.CTRLA = ( USARTD0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;
		break;

	}

	portEXIT_CRITICAL();

	return;
}
//----------------------------------------------------------------------------
void drvUART_InterruptOn( const int UARTx )
{

	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA;

	switch(UARTx) {
	case pUSB:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------
void drvUART_InterruptOff( const int UARTx )
{
uint8_t tempCTRLA;

	switch(UARTx) {
	case pUSB:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}
}
//----------------------------------------------------------------------------
ISR(USARTD0_DRE_vect)
{

int8_t cTaskWoken;
char cChar;
int8_t res = pdFALSE;

	res = xQueueReceiveFromISR( spd_USB.uart.txQueue, &cChar, &cTaskWoken );

	if( res == pdTRUE ) {
		/* Send the next character queued for Tx. */
		USARTD0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		drvUART_InterruptOff( pUSB);
	}
}
//----------------------------------------------------------------------------
ISR(USARTD0_RXC_vect)
{

char cChar;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTD0.DATA;

	if( xQueueSendFromISR( spd_USB.uart.rxQueue, &cChar, 0 ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------