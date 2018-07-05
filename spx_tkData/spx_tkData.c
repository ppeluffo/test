/*
 * spx_tkData.c
 *
 *  Created on: 5 jul. 2018
 *      Author: pablo
 */

#include "spx.h"

// Este factor es porque la resistencia shunt es de 7.3 por lo que con 20mA llegamos hasta 3646 y no a 4096
#define FACTOR_CORRECCION_RSHUNT	3646

//------------------------------------------------------------------------------------
// PROTOTIPOS

static bool pv_data_guardar_BD( void );
static void pv_data_signal_to_tkgprs(void);

// VARIABLES LOCALES
static st_data_frame pv_data_frame;

// La tarea pasa por el mismo lugar c/timerPoll secs.
#define WDG_DAT_TIMEOUT	 ( systemVars.timerPoll + 60 )

//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;

uint32_t waiting_ticks;
TickType_t xLastWakeTime;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmd_xprintf_P(pUSB,PSTR("starting tkData..\r\n\0"));

	// Configuro los INA para promediar en 128 valores.
	pub_analog_config_INAS(CONF_INAS_AVG128);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Al arrancar poleo a los 10s
    waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

	// loop
	for( ;; )
	{
		pub_ctl_watchdog_kick(WDG_DAT, WDG_DAT_TIMEOUT);

		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		pub_data_read_frame();

		// Muestro en pantalla.
		pub_data_print_frame();

		// Salvo en BD ( si no es el primer frame )
		if ( pv_data_guardar_BD() ) {
			// Aviso a tkGPRS ( si estoy en modo continuo )
			pv_data_signal_to_tkgprs();
		}

		// Espero un ciclo
		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
			taskYIELD();
		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
static bool pv_data_guardar_BD(void)
{

	// Solo los salvo en la BD si estoy en modo normal.
	// En otros casos ( service, monitor_frame, etc, no.

FAT_t l_fat;
int8_t bytes_written;
static bool primer_frame = true;

	// Para no incorporar el error de los contadores en el primer frame no lo guardo.
	if ( primer_frame ) {
		primer_frame = false;
		return(false);
	}

	// Guardo en BD
	bytes_written = FS_writeRcd( &pv_data_frame, sizeof(st_data_frame) );

	if ( bytes_written == -1 ) {
		// Error de escritura o memoria llena ??
		cmd_xprintf_P(pUSB,PSTR("DATA: WR ERROR (%d)\r\n\0"),FS_errno() );
		// Stats de memoria
		FS_fat_read(&l_fat);
		cmd_xprintf_P(pUSB, PSTR("DATA: MEM [wr=%d,rd=%d,del=%d]\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR );
		return(false);
	}

	return(true);

}
//------------------------------------------------------------------------------------
static void pv_data_signal_to_tkgprs(void)
{
	// Aviso a tkGprs que hay un frame listo. En modo continuo lo va a trasmitir enseguida.
	if ( ! MODO_DISCRETO ) {
//		while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
//			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
//		}
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_data_read_frame(void)
{
	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

	// Leo los canales analogicos.
	// Prendo los sensores, espero un settle time de 1s, los leo y apago los sensores.
	ACH_prender_12V();

	pub_analog_config_INAS(CONF_INAS_AVG128);	// Saco a los INA del modo pwr_down
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	pub_analog_read_frame( &pv_data_frame.analog_frame);
	pub_analog_config_INAS(CONF_INAS_SLEEP);	// Pongo a los INA a dormir.
	ACH_apagar_12V();

	// Leo la bateria
	pub_analog_read_battery ( &pv_data_frame.battery );

	// Leo los canales digitales y borro los contadores.
	pub_digital_read_frame( &pv_data_frame.digital_frame, true );

	// Agrego el timestamp
	RTC_read_dtime( &pv_data_frame.rtc);

}
//------------------------------------------------------------------------------------
void pub_data_print_frame(void)
{
	// Imprime el frame actual en consola

uint8_t channel;

	// HEADER
	cmd_xprintf_P(pUSB,PSTR("frame: " ) );
	// timeStamp.
	cmd_xprintf_P(pUSB,PSTR( "%04d%02d%02d,"),pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day );
	cmd_xprintf_P(pUSB,PSTR("%02d%02d%02d"),pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		cmd_xprintf_P( pUSB, PSTR(",%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
	}

	// Valores digitales. Lo que mostramos depende de lo que tenemos configurado
	// Niveles logicos.
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		// Si el canal no esta configurado no lo muestro.
		if ( ! strcmp ( systemVars.d_ch_name[channel], "X" ) ) {
			continue;
		}
		// Level ?
		if ( systemVars.d_ch_type[channel] == 'L') {
			cmd_xprintf_P(pUSB, PSTR(",%s=%d"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.level[channel] );
		} else {
		// Counter ?
			cmd_xprintf_P(pUSB, PSTR(",%s=%.02f"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.magnitud[channel] );
		}
	}

	// bateria
	cmd_xprintf_P(pUSB, PSTR(",BAT=%.02f"), pv_data_frame.battery );

	// TAIL
	cmd_xprintf_P(pUSB, PSTR("\r\n\0") );

}
//------------------------------------------------------------------------------------
