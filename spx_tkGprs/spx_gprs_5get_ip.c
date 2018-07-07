/*
 * sp5KV5_tkGprs_ask_ip.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spx_gprs.h"

static bool pv_gprs_netopen(void);
static bool pg_gprs_activate(void);
static void pv_gprs_read_ip_assigned(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_IP	180

//------------------------------------------------------------------------------------
bool st_gprs_get_ip(void)
{
	// El modem esta prendido y configurado.
	// Intento hasta 3 veces pedir la IP.
	// WATCHDOG: En el peor caso demoro 2 mins.
	// La asignacion de la direccion IP es al activar el contexto con el comando AT+CGACT

bool exit_flag = bool_RESTART;

// Entry:
	GPRS_stateVars.state = G_GET_IP;
	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_IP);

	//if ( pg_gprs_activate() ) {
	if ( pv_gprs_netopen() ) {
		exit_flag = bool_CONTINUAR;
	} else {
		// Aqui es que luego de tantos reintentos no consegui la IP.
		exit_flag = bool_RESTART;
		cmd_xprintf_P(pUSB,PSTR("GPRS: ERROR: ip no asignada !!.\r\n\0") );
	}

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pg_gprs_activate(void)
{
	/* Intento varias veces activar el APN para que me de una IP.
	 *
	 */

uint8_t net_tryes, qry_tryes;

	cmd_xprintf_P(pUSB,PSTR("GPRS: NET activation (get IP).\r\n\0"));

	// Intento MAX_IP_QUERIES veces que me asignen una IP.
	// AT+CGACT=1,1
	for ( net_tryes = 0; net_tryes < MAX_IP_QUERIES; net_tryes++ ) {

		// Envio el comando para activar el contexto y que me de una IP.
		pub_gprs_flush_RX_buffer();
		xprintf_P(pGPRS,PSTR("AT+CGACT=1,1\r\0"));
		vTaskDelay( ( TickType_t)( 3000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		for ( qry_tryes = 0; qry_tryes < 5; qry_tryes++ ) {

			// Envio el comando para ver si se activo el contexto
			pub_gprs_flush_RX_buffer();
			xprintf_P(pGPRS,PSTR("AT+CGACT?\r\0"));
			vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			}

			// Analizo la respuesta
			if ( pub_gprs_check_response("+CGACT: 1\0")) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: ip address OK.\r\n\0") );
				pv_gprs_read_ip_assigned();	// Leo e informo cual IP me asigno la red
				return(true);
			}

			if ( pub_gprs_check_response("ERROR\0")) {
				// Error: salgo del loop de espera y voy a reintentar dar el comando
				return(false);
			}

			// Espero 5s antes de consultar de nuevo
			vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );
		}

	}

	return(false);
}
//------------------------------------------------------------------------------------
static bool pv_gprs_netopen(void)
{
	// Doy el comando para atachearme a la red
	// Puede demorar unos segundos por lo que espero para chequear el resultado
	// y reintento varias veces.

uint8_t reintentos = MAX_TRYES_NET_ATTCH;
uint8_t checks;

	cmd_xprintf_P(pUSB,PSTR("GPRS: netopen (get IP).\r\n\0") );

	while ( reintentos-- > 0 ) {

		// AT+NETOPEN
		// Espero 2s para dar el comando
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		pub_gprs_flush_RX_buffer();
		if ( systemVars.debug == DEBUG_GPRS ) {
			cmd_xprintf_P(pUSB,PSTR("GPRS: send NETOPEN cmd (%d)\r\n\0"),reintentos );
		}
		xprintf_P(pGPRS,PSTR("AT+NETOPEN\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

		// Intento 3 veces ver si respondio correctamente.
		for ( checks = 0; checks < 5; checks++) {

			if ( systemVars.debug == DEBUG_GPRS ) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: netopen check.(%d)\r\n\0"),checks );
			}

			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			}

			if ( pub_gprs_check_response("+NETOPEN: 0")) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: NETOPEN OK !.\r\n\0") );
				pv_gprs_read_ip_assigned();
				return(true);
			}

			if ( pub_gprs_check_response("ERROR")) {
				break;	// Salgo del for
			}

			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
		}

		// No pude atachearme. Debo mandar de nuevo el comando
	}

	// Luego de varios reintentos no pude conectarme a la red.
	cmd_xprintf_P(pUSB,PSTR("GPRS: NETOPEN FAIL !!.\r\n\0"));
	return(false);

}
//------------------------------------------------------------------------------------
static void pv_gprs_read_ip_assigned(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress

char *ts = NULL;
int i=0;
char c;

	// AT+CGPADDR para leer la IP
	pub_gprs_flush_RX_buffer();
	//FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGPADDR\r\0"));
	xprintf_P(pGPRS,PSTR("AT+IPADDR\r\0"));
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Extraigo la IP del token. Voy a usar el buffer  de print ya que la respuesta
	// puede ser grande.
	memcpy(gprs_printfBuff, pub_gprs_rxbuffer_getPtr(), sizeof(gprs_printfBuff) );

	ts = strchr( gprs_printfBuff, ':');
	ts++;
	while ( (c= *ts) != '\r') {
		systemVars.dlg_ip_address[i++] = c;
		ts++;
	}
	systemVars.dlg_ip_address[i++] = '\0';

	cmd_xprintf_P(pUSB,PSTR("GPRS: ip address=[%s]\r\n\0"), systemVars.dlg_ip_address);

}
//------------------------------------------------------------------------------------
