/*
 * sp5KV5_tkGprs_init_frame.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spx_gprs.h"

static bool pv_send_init_frame(void);
static bool pv_process_init_response(void);
static void pv_TX_init_frame(void);
static void pv_process_server_clock(void);
static void pv_reconfigure_params(void);

static uint8_t pv_process_pwrSave(void);
static uint8_t pv_process_timerPoll(void);
static uint8_t pv_process_timerDial(void);
static uint8_t pv_process_digitalCh(uint8_t channel);
static uint8_t pv_process_AnalogCh(uint8_t channel);
static uint8_t pv_process_Outputs(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_INIT	180

//------------------------------------------------------------------------------------
bool st_gprs_init_frame(void)
{
	// Debo mandar el frame de init al server, esperar la respuesta, analizarla
	// y reconfigurarme.
	// Intento 3 veces antes de darme por vencido.
	// El socket puede estar abierto o cerrado. Lo debo determinar en c/caso y
	// si esta cerrado abrirlo.
	// Mientras espero la respuesta debo monitorear que el socket no se cierre

uint8_t intentos;
bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_INIT_FRAME;

	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_INIT );

	cmd_xprintf_P(pUSB,PSTR("GPRS: iniframe.\r\n\0" ));

	// Intenteo MAX_INIT_TRYES procesar correctamente el INIT
	for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

		if ( pv_send_init_frame() && pv_process_init_response() ) {		// Intento madar el frame al servidor
			// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
			if ( systemVars.debug == DEBUG_GPRS ) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: Init frame OK.\r\n\0" ));
			}

			exit_flag = true;
			goto EXIT;

		} else {

			if ( systemVars.debug == DEBUG_GPRS ) {
				cmd_xprintf_P(pUSB,PSTR("GPRS: iniframe retry(%d)\r\n\0"),intentos);
			}

			// Espero 3s antes de reintentar
			vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );
		}
	}

	// Aqui es que no puede enviar/procesar el INIT correctamente
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Init frame FAIL !!.\r\n\0" ));
	}

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_send_init_frame(void)
{
	// Intento enviar 1 SOLO frame de init.
	// El socket puede estar cerrado por lo que reintento abrirlo hasta 3 veces.
	// Una vez que envie el INIT, salgo.
	// Al entrar, veo que el socket este cerrado.

uint8_t i;
bool exit_flag = false;
uint8_t timeout;

	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		if (  pub_gprs_check_socket_status() == SOCK_OPEN ) {
			pv_TX_init_frame();		// Escribo en el socket el frame de INIT
			exit_flag = true;
			break;
		}

		// Doy el comando para abrirlo.
		pub_gprs_open_socket();

		// Y espero hasta 10s que abra.
		for ( timeout = 0; timeout < 10; timeout++) {
			vTaskDelay( (portTickType)( 3000 / portTICK_RATE_MS ) );
			if ( pub_gprs_check_socket_status() == SOCK_OPEN )
				break;
		}
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
static bool pv_process_init_response(void)
{
	// Espero la respuesta al frame de INIT.
	// Si la recibo la proceso.
	// Salgo por timeout 10s o por socket closed.

uint8_t timeout;
bool exit_flag = false;

	for ( timeout = 0; timeout < 10; timeout++) {

		vTaskDelay( (portTickType)( 1500 / portTICK_RATE_MS ) );				// Espero 1s

		if ( pub_gprs_check_socket_status() == SOCK_CLOSED ) {		// El socket se cerro
			exit_flag = false;
			goto EXIT;
		}

		if ( pub_gprs_check_response("ERROR") ) {	// Recibi un ERROR de respuesta
			pub_gprs_print_RX_Buffer();
			exit_flag = false;
			goto EXIT;
		}

		if ( pub_gprs_check_response("INIT_OK") ) {	// Respuesta correcta del server
			if ( systemVars.debug == DEBUG_GPRS  ) {
				pub_gprs_print_RX_Buffer();
			} else {
				pub_gprs_print_RX_response();
			}
			pv_reconfigure_params();
			exit_flag = true;
			goto EXIT;
		}

	}

// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_TX_init_frame(void)
{
	// SP5KV5_3CH
	// Send Init Frame
	// GET /cgi-bin/sp5K/sp5K.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&ALARM&PWRM=CONT&TPOLL=23&TDIAL=234&PWRS=1,1230,2045&A0=pZ,1,20,3,10&D0=qE,3.24&CONS=1,1234,927,1,3 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

	// SP5KV5_8CH
	// Send Init Frame
	// GET /cgi-bin/sp5K8CH.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&CSQ=75 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

uint16_t pos = 0;
uint8_t i;

	if ( systemVars.debug == DEBUG_GPRS  ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: iniframe: Sent\r\n\0"));
	}

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	pub_gprs_flush_RX_buffer();

	// GPRS sent
	xprintf_P(pGPRS,PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=%s&INIT1&TPOLL=%d&TDIAL=%d&PWRS=%d,%02d%02d,%02d%02d&OUTS=%d,%02d%02d,%02d%02d&CSQ=%d\0"), systemVars.serverScript, systemVars.dlgId, &buff_gprs_imei, SPX_FW_REV,systemVars.timerPoll, systemVars.timerDial,systemVars.pwrSave.modo,systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min,systemVars.outputs.modo,systemVars.outputs.consigna_diurna.hour,systemVars.outputs.consigna_diurna.min,systemVars.outputs.consigna_nocturna.hour,systemVars.outputs.consigna_nocturna.min, systemVars.dbm);
	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );

	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GET %s?DLGID=%s&PASSWD=%s&IMEI=%s&VER=%s&INIT&TPOLL=%d&TDIAL=%d&PWRS=%d,%02d%02d,%02d%02d&OUTS=%d,%02d%02d,%02d%02d&CSQ=%d\r\n0"), systemVars.serverScript, systemVars.dlgId, &buff_gprs_imei, SPX_FW_REV,systemVars.timerPoll, systemVars.timerDial,systemVars.pwrSave.modo,systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min,systemVars.outputs.modo,systemVars.outputs.consigna_diurna.hour,systemVars.outputs.consigna_diurna.min,systemVars.outputs.consigna_nocturna.hour,systemVars.outputs.consigna_nocturna.min, systemVars.dbm);
		vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );
	}


	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	xprintf_P( pGPRS,PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0"));
	vTaskDelay( (portTickType)( 250 / portTICK_RATE_MS ) );

	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n\0"));
	}

}
//------------------------------------------------------------------------------------
static void pv_reconfigure_params(void)
{

uint8_t saveFlag = 0;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	pv_process_server_clock();

	saveFlag += pv_process_timerPoll();
	saveFlag += pv_process_timerDial();
	saveFlag += pv_process_pwrSave();

	// Canales analogicos.
	saveFlag += pv_process_AnalogCh(0);
	saveFlag += pv_process_AnalogCh(1);
	saveFlag += pv_process_AnalogCh(2);
	saveFlag += pv_process_AnalogCh(3);
	saveFlag += pv_process_AnalogCh(4);

	// Canales digitales
	saveFlag += pv_process_digitalCh(0);
	saveFlag += pv_process_digitalCh(1);
	saveFlag += pv_process_digitalCh(2);
	saveFlag += pv_process_digitalCh(3);

	// Outputs/Consignas
	saveFlag += pv_process_Outputs();

	if ( saveFlag > 0 ) {

		pub_save_params_in_NVMEE();

		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_GPRS ) {
			cmd_xprintf_P(pUSB,PSTR("GPRS: Save params OK\r\n\0"));
		}
	}

}
//------------------------------------------------------------------------------------
static void pv_process_server_clock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p, *s;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char rtcStr[12];
uint8_t i;
char c;
RtcTimeType_t rtc;

	s = pub_gprs_rxbuffer_getPtr();
	p = strstr(s, "CLOCK");
	if ( p == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK

	token = strsep(&stringp,delim);			// rtc
	memset(rtcStr, '\0', sizeof(rtcStr));
	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
	for ( i = 0; i<12; i++) {
		c = *token;
		rtcStr[i] = c;
		c = *(++token);
		if ( c == '\0' )
			break;

	}

	RTC_str2rtc(rtcStr, &rtc);		// Convierto el string YYMMDDHHMM a RTC.
	RTC_write_dtime(&rtc);		// Grabo el RTC

	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Update rtc to: %s\r\n\0"), rtcStr );
	}

}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = pub_gprs_rxbuffer_getPtr();
	p = strstr(s, "TPOLL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll

	pub_analog_config_timerpoll(token);

	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig TPOLL\r\n\0"));
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerDial(void)
{
	//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:CD=1230:CN=0530</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = pub_gprs_rxbuffer_getPtr();
	p = strstr(s, "TDIAL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TDIAL

	token = strsep(&stringp,delim);	// timerDial

	pub_gprs_config_timerdial(token);
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig TDIAL\r\n\0"));
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_pwrSave(void)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRS=1,2230,0600:D0=q0,1.00:D1=q1,1.00</h1>
//  Las horas estan en formato HHMM.

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *p1,*p2;
uint8_t modo;
char *p, *s;

	s = pub_gprs_rxbuffer_getPtr();
	p = strstr(s, "PWRS");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//PWRS

	token = strsep(&stringp,delim);	// modo
	modo = atoi(token);
	p1 = strsep(&stringp,delim);	// startTime
	p2 = strsep(&stringp,delim); 	// endTime

	pub_configPwrSave(modo, p1, p2);
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig PWRSAVE\r\n\0"));
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_AnalogCh(uint8_t channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName,*s_iMin,*s_iMax,*s_mMin,*s_mMax;
char *s;

	s = pub_gprs_rxbuffer_getPtr();

	switch (channel) {
	case 0:
		stringp = strstr(s, "A0=");
		break;
	case 1:
		stringp = strstr(s, "A1=");
		break;
	case 2:
		stringp = strstr(s, "A2=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//A0

	chName = strsep(&stringp,delim);	//name
	s_iMin = strsep(&stringp,delim);	//iMin
	s_iMax = strsep(&stringp,delim);	//iMax
	s_mMin = strsep(&stringp,delim);	//mMin
	s_mMax = strsep(&stringp,delim);	//mMax

	pub_analog_config_channel( channel, chName,s_iMin,s_iMax,s_mMin,s_mMax );
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig A%d\r\n\0"), channel);
	}

quit:
	return(ret);
}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_digitalCh(uint8_t channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=C,q0,1.00:D1=L,q1</h1>
//
// D0=C,q0,1.00:D1=L,q1
//
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chType, *chName, *s_magPP;
char *s;

	s = pub_gprs_rxbuffer_getPtr();
	switch (channel) {
	case 0:
		stringp = strstr(s, "D0=");
		break;
	case 1:
		stringp = strstr(s, "D1=");
		break;
	default:
		ret = 0;
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		// No viene configuracion de D0 ni de D1.
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado ( solo 32 bytes ) a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp, 31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//D0

	chType = strsep(&stringp,delim);	//tipo
	chName = strsep(&stringp,delim);	//name
	s_magPP = strsep(&stringp,delim);	//magPp
	pub_digital_config_channel( channel, chType, chName, s_magPP );
	ret = 1;
	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig D%d\r\n\0"), channel);
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_Outputs(void)
{
	//	La linea recibida es del tipo:
	//	<h1>INIT_OK:OUTS=modo,param1,param2:</h1>
	// modo=0: OUTS_OFF
	// modo=1: OUTS_NORMAL. param1,param2 indican los valores de las salidas
	// modo=2: OUTS_CONSIGNAS: param1, param2 son las horas de la consigna diurna y la nocturna
	//  Las horas estan en formato HHMM.

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *modo,*p1,*p2;

char *p, *s;

	s = pub_gprs_rxbuffer_getPtr();
	p = strstr(s, "OUTS");
	if ( p == NULL )
		goto quit;

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	//OUTS

	modo = atoi(strsep(&stringp,delim));	// modo
	p1 = strsep(&stringp,delim);			// startTime
	p2 = strsep(&stringp,delim); 			// endTime

//	pub_outputs_config(modo, p1, p2);
	ret = 1;

	if ( systemVars.debug == DEBUG_GPRS ) {
		cmd_xprintf_P(pUSB,PSTR("GPRS: Reconfig OUTPUTS (modo=%d,p1=%s,p2=%s)\r\n\0"), modo, p1,p2);
	}

quit:
	return(ret);

}
//--------------------------------------------------------------------------------------
