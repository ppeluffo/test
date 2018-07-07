/*
 * drv_uart_spx.c
 *
 *  Created on: 28 jun. 2018
 *      Author: pablo
 */


#include "drv_uart_spx.h"

//----------------------------------------------------------------------------
void drv_UART_baud_ctl(uint8_t *baudA, uint8_t *baudB, uint8_t *ctl)
{
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
	*baudA = (uint8_t) 2094;
	*baudB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
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
	*baudA = (uint8_t) 983;
	*baudB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
	*ctl |= USART_CLK2X_bm;
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
		*baudA = (uint8_t) 11;
		*baudB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
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
	case pGPRS:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTE0.CTRLA = tempCTRLA;
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
	case pGPRS:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	}
}
//----------------------------------------------------------------------------
// UART_USB: PORTD
//----------------------------------------------------------------------------
void drvUART_USB_open( const uint32_t flags )
{
	// El puerto del USB es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	portENTER_CRITICAL();

	PORTD.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
	PORTD.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */
	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTD0.CTRLB;
	drv_UART_baud_ctl(&baudA, &baudB, &ctl);
	USARTD0.BAUDCTRLA = baudA;
	USARTD0.BAUDCTRLB = baudB;
	USARTD0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTD0.CTRLB |= USART_RXEN_bm;
	USARTD0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTD0.CTRLA = ( USARTD0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	portEXIT_CRITICAL();

	return;
}
//----------------------------------------------------------------------------
ISR(USARTD0_DRE_vect)
{

int8_t cTaskWoken;
char cChar;
bool res = false;

	res = xQueueReceiveFromISR( spd_USB.uart.txQueue, &cChar, &cTaskWoken );

	if( res == true ) {
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
	xQueueSendFromISR( spd_USB.uart.rxQueue, &cChar, 0 );

}
//----------------------------------------------------------------------------
// UART_GPRS: PORTE
//----------------------------------------------------------------------------
void drvUART_GPRS_open( const uint32_t flags )
{
	// El puerto del GPRS es PORTE:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	portENTER_CRITICAL();

	PORTE.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
	PORTE.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */
	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USARTE0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

//	ctl = USARTE0.CTRLB;
//	drv_UART_baud_ctl(&baudA, &baudB, &ctl);
//	USARTE0.BAUDCTRLA = baudA;
//	USARTE0.BAUDCTRLB = baudB;

	USARTE0.BAUDCTRLA = (uint8_t) 2094;
	USARTE0.BAUDCTRLB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
//	USARTE0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTE0.CTRLB = 0x00;
	USARTE0.CTRLB |= USART_RXEN_bm;
	USARTE0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTE0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTE0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTE0.CTRLA = ( USARTE0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	portEXIT_CRITICAL();

	return;
}
//----------------------------------------------------------------------------
ISR(USARTE0_DRE_vect)
{

int8_t cTaskWoken;
char cChar;
bool res = false;

	res = xQueueReceiveFromISR( spd_GPRS.uart.txQueue, &cChar, &cTaskWoken );

	if( res == true ) {
		/* Send the next character queued for Tx. */
		USARTE0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		drvUART_InterruptOff( pGPRS);
	}
}
//----------------------------------------------------------------------------
ISR(USARTE0_RXC_vect)
{

char cChar;

	cChar = USARTE0.DATA;
	xQueueSendFromISR( spd_GPRS.uart.rxQueue, &cChar, 0 );
	//xQueueOverwriteFromISR( spd_GPRS.uart.rxQueue, &cChar, 0 );
}
//----------------------------------------------------------------------------
// BT_USB: PORTD
//----------------------------------------------------------------------------
void drvUART_BT_open( const uint32_t flags )
{
	// El puerto del USB es PORTF:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA, baudB, ctl;

	portENTER_CRITICAL();

	PORTF.DIRSET   = PIN3_bm;	/* PD3 (TXD0) as output. */
	PORTF.DIRCLR   = PIN2_bm;	/* PD2 (RXD0) as input. */
	/* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
	USARTF0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTF0.CTRLB;
	drv_UART_baud_ctl(&baudA, &baudB, &ctl);
	USARTF0.BAUDCTRLA = baudA;
	USARTF0.BAUDCTRLB = baudB;
	USARTF0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTF0.CTRLB |= USART_RXEN_bm;
	USARTF0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTF0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTF0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTF0.CTRLA = ( USARTF0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	portEXIT_CRITICAL();

	return;
}
//----------------------------------------------------------------------------
ISR(USARTF0_DRE_vect)
{

int8_t cTaskWoken;
char cChar;
bool res = false;

	res = xQueueReceiveFromISR( spd_BT.uart.txQueue, &cChar, &cTaskWoken );

	if( res == true ) {
		/* Send the next character queued for Tx. */
		USARTF0.DATA = cChar;
	} else {
		/* Queue empty, nothing to send. */
		drvUART_InterruptOff( pBT);
	}
}
//----------------------------------------------------------------------------
ISR(USARTF0_RXC_vect)
{

char cChar;

	/* Get the character and post it on the queue of Rxed characters.
	 * If the post causes a task to wake force a context switch as the woken task
	 * may have a higher priority than the task we have interrupted.
    */
	cChar = USARTF0.DATA;
	xQueueSendFromISR( spd_BT.uart.rxQueue, &cChar, 0 );

}
//----------------------------------------------------------------------------
