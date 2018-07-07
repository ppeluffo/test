/*
 * sp5KV5_tkGprs_prender.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 *
 *  En este estado prendo el modem asegurandome que lo esta al responder a un
 *  comando AT con un OK.
 *  Luego leo el IMEI.
 *  El dlg sale del modo tickless tanto en TX como RX al prender la flag de modem_prendido.
 *
 */

#include "spx_gprs.h"

static void pv_gprs_readImei(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_PRENDER	180

//------------------------------------------------------------------------------------
bool st_gprs_prender(void)
{
	// Intento prender el modem hasta 3 veces. Si no puedo, fijo el nuevo tiempo
	// para esperar y salgo.
	// Mientras lo intento prender no atiendo mensajes ( cambio de configuracion / flooding / Redial )

uint8_t hw_tries, sw_tries;
bool exit_flag = bool_RESTART;


	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_PRENDER);

	GPRS_stateVars.state = G_PRENDER;

	// Debo poner esta flag en true para que el micro no entre en sleep y pueda funcionar el puerto
	// serial y leer la respuesta del AT del modem.
	GPRS_stateVars.modem_prendido = true;
	vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );

	cmd_xprintf_P(pUSB,PSTR("GPRS: prendo modem...\r\n\0"));

// Loop:
	for ( hw_tries = 0; hw_tries < MAX_HW_TRIES_PWRON; hw_tries++ ) {

		pub_gprs_modem_pwr_on();

		// Reintento prenderlo activando el switch pin
		for ( sw_tries = 0; sw_tries < MAX_SW_TRIES_PWRON; sw_tries++ ) {

			if ( systemVars.debug == DEBUG_GPRS ) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: intentos: HW=%d,SW=%d\r\n\0"), hw_tries, sw_tries);
			}

			// Genero el toggle del switch pin para prenderlo.
			pub_gprs_modem_pwr_sw();
			pv_gprs_rxbuffer_flush();

			// Espero 10s para interrogarlo
			vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );

			// Mando un AT y espero un OK para ver si prendio y responde.
			pub_gprs_flush_RX_buffer();
			xprintf_P(pGPRS, PSTR("AT\r\0"));
			vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			}

			if ( pub_gprs_check_response("OK\0") ) {
				// Respondio OK. Esta prendido; salgo
				cmd_xprintf_P(pUSB,PSTR("GPRS: Modem on.\r\n\0"));
				exit_flag = bool_CONTINUAR;
				goto EXIT;
			} else {
				if ( systemVars.debug == DEBUG_GPRS ) {
					cmd_xprintf_P(pUSB,PSTR("GPRS: Modem No prendio!!\r\n\0"));
				}
			}

		}

		// No prendio luego de MAX_SW_TRIES_PWRON intentos SW. Apago y prendo de nuevo
		pub_gprs_modem_pwr_on();									// Apago la fuente
		vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );	// Espero 10s antes de reintentar
	}

	// Si salgo por aqui es que el modem no prendio luego de todos los reintentos
	exit_flag = bool_RESTART;
	cmd_xprintf_P(pUSB,PSTR("GPRS: ERROR!! Modem no prendio en HW%d y SW%d intentos\r\n\0"), MAX_HW_TRIES_PWRON, MAX_SW_TRIES_PWRON);

	// Exit:
EXIT:

	// Ajusto la flag modem_prendido ya que termino el ciclo y el micro pueda entrar en sleep.
	if ( exit_flag == bool_CONTINUAR ) {
		pv_gprs_readImei();		// Leo el IMEI
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_gprs_readImei(void)
{
	// Leo el imei del modem para poder trasmitirlo al server y asi
	// llevar un control de donde esta c/sim

uint8_t i,j,start, end;

	// Envio un AT+CGSN para leer el IMEI
	pub_gprs_flush_RX_buffer();
//	memset(gprs_printfBuff,'\0', sizeof(gprs_printfBuff));
	xprintf_P( pGPRS, PSTR("AT+CGSN\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Leo y Evaluo la respuesta al comando AT+CGSN
	if ( pub_gprs_check_response("OK\0") ) {
		// Extraigoel IMEI del token. Voy a usar el buffer  de print ya que la respuesta
		// puede ser grande.
		memcpy(gprs_printfBuff, pub_gprs_rxbuffer_getPtr(), sizeof(gprs_printfBuff) );

		// Guardo el IMEI
		start = 0;
		end = 0;
		j = 0;
		// Busco el primer digito
		for ( i = 0; i < 64; i++ ) {
			if ( isdigit( gprs_printfBuff[i]) ) {
				start = i;
				break;
			}
		}
		if ( start == end )		// No lo pude leer.
			goto EXIT;

		// Busco el ultimo digito y copio todos
		for ( i = start; i < 64; i++ ) {
			if ( isdigit( gprs_printfBuff[i]) ) {
				buff_gprs_imei[j++] = gprs_printfBuff[i];
			} else {
				break;
			}
		}
	}

// Exit
EXIT:

	cmd_xprintf_P(pUSB,PSTR("GPRS: IMEI[%s]\r\n\0"),buff_gprs_imei);

}
//--------------------------------------------------------------------------------------
