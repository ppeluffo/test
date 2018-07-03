/*
 * main.c
 *
 *  Created on: 18 de oct. de 2016
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:spx.hex
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  REFERENCIA: /usr/lib/avr/include/avr/iox256a3b.h
 *
 *  El watchdog debe estar siempre prendido por fuses.
 *  FF 0A FD __ F5 D4
 *
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n test_io.elf | more
 *
 *------------------------------------------------------------------------------------------
 * 2018-07-02:
 *  - Pasamos a FRTOS10. Excluimos timers.c, mpu_wrappers.c, etc.
 *  - Usamos el heap.c.1 de modo que la memoria consumida se ve en el plano de memoria
 *  - Modificamos el port.c para que el save/restore sea de 3 bytes y use el modo tickeless.
 *  - Pasamos a usar string_buffers para la recepcion de las uarts.
 *    El tamaño de las queues es de UBaseType_t(1byte) en cambio el de las string_buffers es
 *    del tipo size_t(2 bytes)
 *  - Creamos una libreria l_queues para poder usar colas de mas de 255 bytes.
 *  - En el frtos_io, manejamos c/dispositivo con funciones propias. La generalidad la damos con
 *    las funciones frtos_io, pero c/puerto luego lo manejamos en forma individual.
 *    En particular para las UARTs usamos UART_queue con lo que dejamos abierto a una posible variante.
 *  - Modifico el driver I2C para generar un codigo de error y eliminar los mensajes.
 *    La idea es que los drivers no generen mensajes.
 *  - En c/tarea saco las variables de las funciones para que al ser externas tenga mas control
 *    de la profundidad del stack.
 *
 *------------------------------------------------------------------------------------------
 *	Crear un proyecto donde simplifique el IO
 *		Revisar secuencia de valores devueltos, profundidad del stack y variables usadas locales.
 *		Revisar tiempos de espera en sFRTOS_UART_read.
 *	Usar buffers y estructuras estaticas
 *	Diagrama de drivers y librerias
 *	Ver uso de queues mas chicas que los buffers
 *	Esquema de control de acceso a buffers comunes ( uart, printf)
 *	Esquema generico del programa en forma modular.
 *
 *------------------------------------------------------------------------------------------
 * 2018-06-27:
 * - Trasmito en los init la potencia del modem en DBM y no CSQ
 * - No trasmito la configuracion de los canales que estan apagados
 *
 * - Las lineas del CGI terminan en \r\n y no en \n. ( Apache 2.4)
 * 	https://en.wikipedia.org/wiki/HTTP_message_body
 * 	http://forum.arduino.cc/index.php?topic=200753.0
 * 	https://www3.ntu.edu.sg/home/ehchua/programming/webprogramming/HTTP_Basics.html
 * 	https://stackoverflow.com/questions/5757290/http-header-line-break-style
 *
 *------------------------------------------------------------------------------------------
 * SCHEDULE:
 * *********
 *  Ampliar el buffer de GPRS_TX
 *
 * HACER:
 * Leer Memoria BD
 *
 * NOTAS DE DISENO:
 * ****************
 *  1- En FRTOS_Write_UART cambio el taskYIELD por taskDelay porque sino se cuelga.
 *     Este delay hacia que los mensajes del cmdmode fuesen lentos y entonces cambio en cmdline.c
 *     la forma de mostrarlos usando directamente FRTOS-IO.
 *
 * PENDIENTE:
 * **********
 * Hacer andar el watchdog
 * Cambiar la velocidad y reconffigurar el BT
 * Configuro el RTC.
 * Rutinas de calendario.
 *
 *
 * Features V0.0.1
 * - Si la terminal no esta conectada, CMD_write no escribe nada.
 * - El FRTOS siempre opera en modo tickless.
 *
 *  XBEE:
 *  Cuando el modo es slave, no uso la tarea de GPRS ( siempre duerme ) y transmito
 *  el frame al remoto.
 *  En esta version, los canales de slave se mapean en los del remoto de modo que el
 *  servidor no se entera de donde vienen.
 
 */


#include "spx.h"

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
//uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
//void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
/*
void wdt_init(void)
{
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );

    return;
}
*/
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
int main( void )
{
	// Clock principal del sistema
	u_configure_systemMainClock();
	u_configure_RTC32();

	// Configuramos y habilitamos el watchdog a 8s.
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );
	if ( WDT_IsWindowModeEnabled() )
		WDT_DisableWindowMode();

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	initMCU();

	//FreeRTOS_open(pUART_USB, ( UART_RXFIFO + UART_TXQUEUE ));
	sFRTOS_open(pUSB, 115200 );
	sFRTOS_open(pBT, 115200 );
	sFRTOS_open(pI2C,100);
	sFRTOS_open(pNVM,0);

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutex();
	//FRTOS_stdio_init();
	xprintf_P_init();

	startTask = false;

	xTaskCreate(tkCtl, "CTL", tkCtl_STACK_SIZE, NULL, tkCtl_TASK_PRIORITY,  &xHandle_tkCtl );
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//-----------------------------------------------------------
void vApplicationIdleHook( void )
{
	// Como trabajo en modo tickless no entro mas en modo sleep aqui.
//	if ( sleepFlag == true ) {
//		sleep_mode();
//	}
}
//-----------------------------------------------------------
/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
//------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	// Es invocada si en algun context switch se detecta un stack corrompido !!
	// Cuando el sistema este estable la removemos.
	// En FreeRTOSConfig.h debemos habilitar
	// #define configCHECK_FOR_STACK_OVERFLOW          2

//	FRTOS_snprintf_P( stdout_buff,sizeof(stdout_buff),PSTR("PANIC:%s !!\r\n\0"),pcTaskName);
//	CMD_write(stdout_buff, sizeof(stdout_buff) );
}
//------------------------------------------------------------------------------------

