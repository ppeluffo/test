/*
 * spx_gprs_utils.c
 *
 *  Created on: 5 jul. 2018
 *      Author: pablo
 */


#include "spx.h"

//------------------------------------------------------------------------------------
int32_t pub_gprs_readTimeToNextDial(void)
{

	return(-1);
}
//------------------------------------------------------------------------------------
void pub_gprs_redial(void)
{

}
//------------------------------------------------------------------------------------
void pub_gprs_config_timerdial ( char *s_timerdial )
{

}
//------------------------------------------------------------------------------------
void pub_gprs_load_defaults(void)
{
	systemVars.timerDial = 900;

	strncpy_P( systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"), APN_LENGTH );
	strncpy_P(systemVars.server_ip_address, PSTR("192.168.0.9\0"),IP_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/spx/spx.pl\0"),SCRIPT_LENGTH);

}
//------------------------------------------------------------------------------------
bool pub_gprs_modem_prendido(void)
{

	return(false);
}
//------------------------------------------------------------------------------------


