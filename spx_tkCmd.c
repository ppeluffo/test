/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */
#include  "spx.h"
#include "frtos_io.h"
#include "frtos_cmd.h"
#include "l_rtc.h"
#include "l_eeprom.h"
#include "l_ina.h"
#include "l_anCh.h"
#include "l_nvm.h"
#include "l_outputs.h"
#include "spx_gprs.h"

#define WR_CMD 0
#define RD_CMD 1

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_rwEE(uint8_t cmd_mode );
static void pv_cmd_rwINA(uint8_t cmd_mode );
static void pv_cmd_rwACH(uint8_t cmd_mode );
static void pv_cmd_OUTPUTS( void );
static void pv_cmd_rwGPRS(uint8_t cmd_mode );
static void pv_cmd_rwNVMEE(uint8_t cmd_mode );
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode );

#define WDG_CMD_TIMEOUT	60

RtcTimeType_t rtc;
char aux_buffer[32];
static usuario_t tipo_usuario;

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);
//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

( void ) pvParameters;
uint8_t c;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );

	cmd_xprintf_P(pUSB, PSTR("starting tkCmd..\r\n\0"));

	tipo_usuario = USER_TECNICO;

	// loop
	for( ;; )
	{

		pub_ctl_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);

		// Si no tengo terminal conectada, duermo 5s lo que me permite entrar en tickless.
		if ( ! pub_ctl_terminal_is_on() ) {
			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		} else {

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( sFRTOS_read( pUSB, (char *)&c, 1 ) == 1 ) {
				FRTOS_CMD_process(c);
			}
		}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

//char aux_str[32];
uint8_t channel;
FAT_t l_fat;

//	cmd_xprintf_P(pUSB,PSTR("dlgid: ??\r\n\0") );
//	cmd_xprintf_P(pUSB,PSTR("size_t: %d\r\n\0"), sizeof(size_t) );
//	cmd_xprintf_P(pUSB,PSTR("unsigned int: %d\r\n\0"), sizeof(unsigned int) );
//	cmd_xprintf_P(pUSB,PSTR("int: %d\r\n\0"), sizeof( int) );
//	cmd_xprintf_P(pUSB,PSTR("port BASE: %d\r\n\0"), sizeof( portBASE_TYPE) );
//	cmd_xprintf_P(pUSB,PSTR("UBaseType_t: %d\r\n\0"), sizeof(UBaseType_t) );
//	cmd_xprintf_P(pUSB,PSTR("uint8_t: %d\r\n\0"), sizeof( uint8_t ) );



	cmd_xprintf_P(pUSB,PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	cmd_xprintf_P(pUSB,PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );

	// SIGNATURE ID
	//memset(&aux_str,'\0', sizeof(aux_str));
	//NVM_readID(aux_str);
	//cmd_xprintf_P(pUSB,PSTR("signature:1 %s\r\n\0"), aux_str);

	// Fecha y Hora
	pv_cmd_rwRTC( RD_CMD );

	// Memoria
	FS_fat_read(&l_fat);
	cmd_xprintf_P(pUSB,PSTR("memory: wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

	// SERVER
	cmd_xprintf_P(pUSB,PSTR(">Server:\r\n\0"));
	cmd_xprintf_P(pUSB,PSTR("  dlgid: %s\r\n\0"), systemVars.dlgId );
	cmd_xprintf_P(pUSB,PSTR("  apn: %s\r\n\0"), systemVars.apn );
	cmd_xprintf_P(pUSB,PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	cmd_xprintf_P(pUSB,PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	cmd_xprintf_P(pUSB,PSTR("  passwd: %s\r\n\0"), systemVars.passwd );

	// MODEM
	cmd_xprintf_P(pUSB,PSTR(">Modem:\r\n\0"));
	cmd_xprintf_P(pUSB,PSTR("  ip address: %s\r\n\0"), systemVars.dlg_ip_address);
	cmd_xprintf_P(pUSB,PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
/*	cmd_xprintf_P(pUSB,PSTR("  state: "));

	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		cmd_xprintf_P(pUSB, PSTR("await_off\r\n"));
		break;
	case G_PRENDER:
		cmd_xprintf_P(pUSB, PSTR("prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		cmd_xprintf_P(pUSB, PSTR("configurando\r\n"));
		break;
	case G_MON_SQE:
		cmd_xprintf_P(pUSB, PSTR("mon_sqe\r\n"));
		break;
	case G_GET_IP:
		cmd_xprintf_P(pUSB, PSTR("ip\r\n"));
		break;
	case G_INIT_FRAME:
		cmd_xprintf_P(pUSB, PSTR("init frame\r\n"));
		break;
	case G_DATA:
		cmd_xprintf_P(pUSB, PSTR("data\r\n"));
		break;
	default:
		cmd_xprintf_P(pUSB, PSTR("ERROR\r\n"));
		break;
	}
*/

	// CONFIG
	cmd_xprintf_P(pUSB, PSTR(">Config:\r\n\0"));
	switch(systemVars.xbee) {
	case XBEE_OFF:
		cmd_xprintf_P(pUSB, PSTR("  xbee: off\r\n\0") );
		break;
	case XBEE_MASTER:
		cmd_xprintf_P(pUSB, PSTR("  xbee: master\r\n\0") );
		break;
	case XBEE_SLAVE:
		cmd_xprintf_P(pUSB, PSTR("  xbee: slave\r\n\0") );
		break;
	}

	switch(systemVars.debug) {
	case DEBUG_NONE:
		cmd_xprintf_P(pUSB, PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_GPRS:
		cmd_xprintf_P(pUSB, PSTR("  debug: gprs\r\n\0") );
		break;
	case DEBUG_RANGEMETER:
		cmd_xprintf_P(pUSB, PSTR("  debug: range\r\n\0") );
		break;
	case DEBUG_DIGITAL:
		cmd_xprintf_P(pUSB, PSTR("  debug: digital\r\n\0") );
		break;
	}

	cmd_xprintf_P(pUSB, PSTR("  timerDial: [%lu s]/%li\r\n\0"),systemVars.timerDial, pub_gprs_readTimeToNextDial() );
	cmd_xprintf_P(pUSB, PSTR("  timerPoll: [%d s]\r\n\0"),systemVars.timerPoll );

	// PULSE WIDTH
	if ( systemVars.rangeMeter_enabled ) {
		cmd_xprintf_P(pUSB,PSTR("  RangeMeter: ON\r\n"));
	} else {
		cmd_xprintf_P(pUSB,PSTR("  RangeMeter: OFF\r\n"));
	}

	// PWR SAVE:
	if ( systemVars.pwrSave.modo ==  modoPWRSAVE_OFF ) {
		cmd_xprintf_P(pUSB, PSTR("  pwrsave=off\r\n\0"));
	} else {
		cmd_xprintf_P(pUSB, PSTR("  pwrsave=on start[%02d:%02d], end[%02d:%02d]\r\n\0"), systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);
	}

	// OUTPUTS:
	switch( systemVars.outputs.modo ) {
	case OUT_OFF:
		cmd_xprintf_P(pUSB, PSTR("  Outputs: OFF\r\n"));
		break;
	case OUT_CONSIGNA:
		switch( systemVars.outputs.consigna_aplicada ) {
		case CONSIGNA_DIURNA:
			cmd_xprintf_P(pUSB, PSTR("  Outputs: CONSIGNA [diurna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		case CONSIGNA_NOCTURNA:
			cmd_xprintf_P(pUSB, PSTR("  Outputs: CONSIGNA [nocturna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		default:
			cmd_xprintf_P(pUSB, PSTR("  Outputs: CONSIGNA [error] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		}
		break;
	case OUT_NORMAL:
		cmd_xprintf_P(pUSB, PSTR("  Outputs: NORMAL (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	default:
		cmd_xprintf_P(pUSB, PSTR("  Outputs: ERROR(%d) (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.modo, systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	}

	// Configuracion de canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			cmd_xprintf_P(pUSB, PSTR("  a%d(*)\0"),channel );
		} else {
			cmd_xprintf_P(pUSB, PSTR("  a%d( )\0"),channel );
		}
		cmd_xprintf_P(pUSB, PSTR(" [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"), systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.an_ch_name[channel] );
	}

	// Configuracion de canales digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		if ( systemVars.d_ch_modo[channel] == 'R') {
			cmd_xprintf_P(pUSB, PSTR("  d%d(*)\0"),channel );
		} else {
			cmd_xprintf_P(pUSB, PSTR("  d%d( )\0"),channel );
		}

		if ( systemVars.d_ch_type[channel] == 'C') {
			// Los canales de contadores de pulsos 'C' muestran el factor de conversion
			cmd_xprintf_P(pUSB, PSTR(" [ C | %s | %.02f ]\r\n\0"), systemVars.d_ch_name[channel],systemVars.d_ch_magpp[channel] );
		} else {
			// Los canales de nivel solo muestran el nombre.
			cmd_xprintf_P(pUSB, PSTR(" [ L | %s ]\r\n\0"), systemVars.d_ch_name[channel] );
		}
	}

	// Valores actuales:
	pub_data_print_frame();

}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {

		// Nadie debe usar la memoria !!!
		vTaskSuspend( xHandle_tkData );

		if (!strcmp_P( strupr(argv[2]), PSTR("SOFT\0"))) {
			FS_format(false );
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("HARD\0"))) {
			FS_format(true );
		}
	}

	cmdClearScreen();

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */

}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// RTC
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr((char *)argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(WR_CMD);
		return;
	}

	// EE
	// write ee pos string
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("EE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// NVMEE
	// write nvmee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwNVMEE(WR_CMD);
		return;
	}

	// RTC SRAM
	// write rtcram pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwRTC_SRAM(WR_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwINA(WR_CMD);
		return;
	}

	// ANALOG
	// write analog {ina_id} conf128
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwACH(WR_CMD);
		return;
	}

	// OUT
	// write out {enable,disable,sleep,awake,set01,set10} {A/B}\r\n\0"));
	// write out pulse (A/B) (ms)\r\n\0"));
	// write out power {on|off}\r\n\0"));
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("OUT\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_OUTPUTS();
		return;
	}

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// CMD NOT FOUND
	cmd_xprintf_P(pUSB,PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

	FRTOS_CMD_makeArgv();

	// WMK
 	if (!strcmp_P( strupr(argv[1]), PSTR("WMK\0"))) {
 		pub_ctl_print_stack_watermarks();
 		return;
 	}

 	// WDT
 	if (!strcmp_P( strupr(argv[1]), PSTR("WDT\0"))) {
 		pub_ctl_print_wdg_timers();
 		return;
 	}

	// RTC
	// read rtc
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		pub_data_read_frame();
		pub_data_print_frame();
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("EE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwNVMEE(RD_CMD);
		return;
	}

	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwRTC_SRAM(RD_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("INA\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwINA(RD_CMD);
		return;
	}

	// ANALOG
	// read analog {ch}, bat
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) && ( tipo_usuario == USER_TECNICO) ) {
		pv_cmd_rwACH(RD_CMD);
		return;
	}

	// UID
	// read uid
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("UID\0"))) {
		cmd_xprintf_P(pUSB, PSTR("UID: %s\r\n\0"), NVMEE_readID() );
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

	// CMD NOT FOUND
	cmd_xprintf_P(pUSB,PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	cmd_xprintf_P(pUSB,PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;

	FRTOS_CMD_makeArgv();

	// USER
	if (!strcmp_P( strupr(argv[1]), PSTR("USER\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("TECNICO\0"))) {
			tipo_usuario = USER_TECNICO;
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("NORMAL\0"))) {
			tipo_usuario = USER_NORMAL;
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// modo
	// config modo {analog|digital} {0..N} {L|R}
	//if (!strcmp_P( strupr(argv[1]), PSTR("MODO\0"))) {
	//	pv_config_modo( argv[2], argv[3], argv[4] );
	//	return;
	//}

	// xbee
	//if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0"))) {
	//	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
	//		systemVars.xbee = XBEE_OFF;
	//		retS = true;
	//	} else if (!strcmp_P( strupr(argv[2]), PSTR("MASTER\0"))) {
	//		systemVars.xbee = XBEE_MASTER;
	//		retS = true;
	//	} else if (!strcmp_P( strupr(argv[2]), PSTR("SLAVE\0"))) {
	//		systemVars.xbee = XBEE_SLAVE;
	//		retS = true;
	//	} else {
	//		retS = false;
	//	}
	//	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
	//	return;
	//}

	// config outputs
	//if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
	//	pub_outputs_config( argv[2], argv[3], argv[4] );
	//	pv_snprintfP_OK();
	//	return;
	//}

	// rangemeter {on|off}
	//if (!strcmp_P( strupr(argv[1]), PSTR("RANGEMETER\0"))) {
	//	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0"))) {
	//		systemVars.rangeMeter_enabled = true;
	//		retS = true;
	//	} else if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
	//		systemVars.rangeMeter_enabled = false;
	//		retS = true;
	//	} else {
	//		retS = false;
	//	}
	//	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
	//	return;
	//}

	// config debug
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("NONE\0"))) {
			systemVars.debug = DEBUG_NONE;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("GPRS\0"))) {
			systemVars.debug = DEBUG_GPRS;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("RANGE\0"))) {
			systemVars.debug = DEBUG_RANGEMETER;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("DIGITAL\0"))) {
			systemVars.debug = DEBUG_DIGITAL;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		pub_save_params_in_NVMEE();
		pv_snprintfP_OK();
		return;
	}

	// config analog {0..2} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) ) {
		pub_analog_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		pv_snprintfP_OK();
		return;
	}

	// config digital {0..3} type dname magPP
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0")) ) {
		if ( pub_digital_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5] ) ) {
			pv_snprintfP_OK();
		} else {
			pv_snprintfP_ERR();
		}
		return;
	}

	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		pub_analog_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config timerdial
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0")) ) {
		pub_gprs_config_timerdial( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config calibrar
	if (!strcmp_P( strupr(argv[1]), PSTR("CFACTOR\0"))) {
		pub_analog_config_spanfactor( atoi(argv[2]), argv[3] );
		pv_snprintfP_OK();
		return;
	}

	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		pub_load_defaults();
		pv_snprintfP_OK();
		return;
	}

	// apn
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// port ( SERVER IP PORT)
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ip (SERVER IP ADDRESS)
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// script ( SERVER SCRIPT SERVICE )
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// passwd
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR( "ON"))) { pub_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { pub_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("WRITE\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-write\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  rtc YYMMDDhhmm\r\n\0"));
		if ( tipo_usuario == USER_TECNICO ) {
			cmd_xprintf_P(pUSB,PSTR("  ee,nvmee,rtcram {pos} {string}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  ina {id} conf {value}, sens12V {on|off}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  analog {ina_id} conf128 \r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  out {enable,disable,sleep,awake,set01,set10} {A/B}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("      pulse (A/B) (ms)\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("      power {on|off}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  gprs (pwr|sw|cts|dtr) {on|off}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("       cmd {atcmd}, redial\r\n\0"));
		}
		return;

	// HELP READ
	} else if (!strcmp_P( strupr( (char *)argv[1]), PSTR("READ\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-read\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  rtc, frame\r\n\0"));
		if ( tipo_usuario == USER_TECNICO ) {
			cmd_xprintf_P(pUSB,PSTR("  ee,nvmee,rtcram {pos} {lenght}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  analog {ch}, bat \r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  uid, nvm {pos} {lenght}\r\n\0"));
			cmd_xprintf_P(pUSB,PSTR("  gprs (rsp,rts,dcd,ri)\r\n\0"));
		}
		return;

	// HELP CONFIG
	} else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-config\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  user {normal|tecnico}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  analog {0..4} aname imin imax mmin mmax\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  cfactor {ch} {coef}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  digital {0..3} type(L,C) dname magPP\r\n\0"));
		//cmd_xprintf_P(pUSB,PSTR("  rangemeter {on|off}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  modo {analog|digital} {0..n} {local|remoto}\r\n\0"));
		//cmd_xprintf_P(pUSB,PSTR("  xbee {off|master|slave}\r\n\0"));
		//cmd_xprintf_P(pUSB,PSTR("  outputs {off}|{normal o0 o1}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  timerpoll, timerdial, dlgid {name}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  pwrsave modo [{on|off}] [{hhmm1}, {hhmm2}]\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  apn, port, ip, script, passwd\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  debug {none,gprs,digital,range}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  default\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  save\r\n\0"));
		return;

	// HELP RESET
	} else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-reset\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  memory {soft|hard}\r\n\0"));
		//cmd_xprintf_P(pUSB,PSTR("  alarm\r\n\0"));
		return;

	// HELP KILL
	} else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-kill {data,digi,gprstx,gprsrx,outputs}\r\n\0"));
		return;

	} else {

	// HELP GENERAL
		cmd_xprintf_P(pUSB,PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		cmd_xprintf_P(pUSB,PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		cmd_xprintf_P(pUSB,PSTR("Available commands are:\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-cls\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-help\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-status\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-reset...\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-kill...\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-write...\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-read...\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("-config...\r\n\0"));
	}

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{
	FRTOS_CMD_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		pub_ctl_watchdog_kick(WDG_DAT, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	// KILL DIGITAL
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGI\0"))) {
		vTaskSuspend( xHandle_tkDigital );
		pub_ctl_watchdog_kick(WDG_DIN, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	// KILL GPRS
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSTX\0"))) {
		vTaskSuspend( xHandle_tkGprsTx );
		pub_ctl_watchdog_kick(WDG_GPRSTX, 0xFFFF);
		// Dejo la flag de modem prendido para poder leer comandos
		GPRS_stateVars.modem_prendido = true;
		pv_snprintfP_OK();
		return;
	}

	// KILL RX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRX\0"))) {
		vTaskSuspend( xHandle_tkGprsRx );
		pub_ctl_watchdog_kick(WDG_GPRSRX, 0xFFFF);
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	cmd_xprintf_P(pUSB,PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	cmd_xprintf_P(pUSB,PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC(uint8_t cmd_mode )
{

bool retS;

	if ( cmd_mode == WR_CMD ) {
		RTC_str2rtc( (char *)argv[2], &rtc);			// Convierto el string YYMMDDHHMM a RTC.
		retS = RTC_write_dtime(&rtc);					// Grabo el RTC
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		RTC_read_dtime(&rtc);
		RTC_rtc2str( aux_buffer,&rtc);
		cmd_xprintf_P(pUSB,PSTR("%s\r\n\0"), aux_buffer );
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwEE(uint8_t cmd_mode )
{

bool retS;
uint8_t length = 0;
char *p;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset( aux_buffer, '\0', sizeof(aux_buffer));
		retS = EE_read( (uint32_t)(atol(argv[2])), &aux_buffer[0], (uint8_t)(atoi(argv[3])) );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			cmd_xprintf_P(pUSB,PSTR("%s\r\n\0"),&aux_buffer[0] );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		retS = EE_write( (uint32_t)(atol(argv[2])), argv[3], length );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwINA(uint8_t cmd_mode )
{

uint16_t val;
uint8_t ina_id;
char data[3];

	// write ina id conf {value}
	if ( cmd_mode == WR_CMD ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0")) ) {
			ina_id = atoi(argv[2]);
			val = atoi( argv[4]);
			data[0] = ( val & 0xFF00 ) >> 8;
			data[1] = ( val & 0x00FF );
			INA_write( ina_id, INA3231_CONF, data, 2 );
			pv_snprintfP_OK();
			return;
		}
	}

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if ( cmd_mode == RD_CMD ) {

		ina_id = atoi(argv[2]);
		if ( ina_id > MAX_INA_ID ) {
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
			INA_read( ina_id, INA3231_CONF, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1SHV\0"))) {
			INA_read ( ina_id, INA3221_CH1_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1BUSV\0"))) {
			INA_read( ina_id, INA3221_CH1_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2SHV\0"))) {
			INA_read( ina_id, INA3221_CH2_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2BUSV\0"))) {
			INA_read(ina_id, INA3221_CH2_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3SHV\0"))) {
			INA_read( ina_id, INA3221_CH3_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3BUSV\0"))) {
			INA_read( ina_id, INA3221_CH3_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("MFID\0"))) {
			INA_read( ina_id, INA3221_MFID, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("DIEID\0"))) {
			INA_read( ina_id, INA3221_DIEID, data, 2 );
		} else {
			pv_snprintfP_ERR();
			return;
		}

		val = ( data[0]<< 8 ) + data	[1];
		cmd_xprintf_P(pUSB,PSTR("VAL=0x%04x\r\n\0"), val);
		pv_snprintfP_OK();
		return;

	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwACH(uint8_t cmd_mode )
{

uint16_t val = 0;
uint8_t channel;

	// read aCh {ch}, bat
	if ( cmd_mode == RD_CMD ) {
		// Bateria
		if (!strcmp_P( strupr( (char *)argv[2]), PSTR("BAT\0"))) {
			val = ACH_read_battery();
			cmd_xprintf_P(pUSB,PSTR("BAT=%d\r\n\0"), val);
			pv_snprintfP_OK();
			return;
		}

		// Canales
		channel = atoi(argv[2]);
		if (  channel > 4 ) {
			pv_snprintfP_ERR();
			return;
		} else {
			val = ACH_read_channel( channel );
			cmd_xprintf_P(pUSB,PSTR("CH%0d=%d\r\n\0"), channel, val);
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// write ach {id} conf128
	if ( cmd_mode == WR_CMD ) {
		ACH_config_avg128(atoi(argv[2]));
		pv_snprintfP_OK();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwNVMEE(uint8_t cmd_mode )
{

bool retS;
uint8_t length = 0;
char *p;

	// read nvmee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset( aux_buffer, '\0', sizeof(aux_buffer));
		retS = NVMEE_read( (uint32_t)(atol(argv[2])), &aux_buffer[0], (uint8_t)(atoi(argv[3])) );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			cmd_xprintf_P(pUSB,PSTR("%s\r\n\0"),&aux_buffer[0] );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write nvm pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		retS = NVMEE_write( (uint32_t)(atol(argv[2])), argv[3], length );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_OUTPUTS( void )
{
	// OUT
	// write out {enable,disable,sleep,awake,set01,set10} {A/B}\r\n\0"));
	// write out pulse (A/B) (ms)\r\n\0"));
	// write out power {on|off}\r\n\0"));

char driver_id;

	driver_id = argv[3][0];

	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("ENABLE\0"))) {
		OUT_driver( driver_id, OUT_ENABLE );
		pv_snprintfP_OK();
		return;

	} else 	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("DISABLE\0"))) {
		OUT_driver( driver_id, OUT_DISABLE );
		pv_snprintfP_OK();
		return;

	} else 	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("SLEEP\0"))) {
		OUT_driver( driver_id, OUT_SLEEP );
		pv_snprintfP_OK();
		return;

	} else 	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("AWAKE\0"))) {
		OUT_driver( driver_id, OUT_AWAKE );
		pv_snprintfP_OK();
		return;

	} else 	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("SET01\0"))) {
		OUT_driver( driver_id, OUT_SET_01 );
		pv_snprintfP_OK();
		return;

	} else 	if (!strcmp_P( strupr( (char *)argv[2]), PSTR("SET10\0"))) {
		OUT_driver( driver_id, OUT_SET_10 );
		pv_snprintfP_OK();
		return;

	}  else {

		pv_snprintfP_ERR();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{

uint8_t pin;
char *p;

	if ( cmd_mode == WR_CMD ) {

		// write gprs (pwr|sw|rts|dtr) {on|off}

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("CTS\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// Por ahora cableo DTR a CTS.

		if (!strcmp_P( strupr(argv[2]), PSTR("DTR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write gprs redial
		if (!strcmp_P( strupr(argv[2]), PSTR("REDIAL\0")) ) {
			pub_gprs_redial();
			return;
		}
		// ATCMD
		// // write gprs cmd {atcmd}
		if (!strcmp_P(strupr(argv[2]), PSTR("CMD\0"))) {

			pub_gprs_flush_RX_buffer();
			xprintf_P(pGPRS,PSTR("%s\r\0"),argv[3] );
			cmd_xprintf_P(pUSB,PSTR("sent->%s\r\n\0"),argv[3] );
			return;
		}

		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp,cts,dcd,ri)

			// ATCMD
			// read gprs rsp
			if (!strcmp_P(strupr(argv[2]), PSTR("RSP\0"))) {
				cmd_xprintf_P(pUSB,PSTR("rx:>%s\r\n\0"), pub_gprs_rxbuffer_getPtr() );
				return;
			}

			// DCD
			if (!strcmp_P( strupr(argv[2]), PSTR("DCD\0")) ) {
				pin = IO_read_DCD();
				cmd_xprintf_P(pUSB,PSTR("DCD=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RI
			if (!strcmp_P( strupr(argv[2]), PSTR("RI\0")) ) {
				pin = IO_read_RI();
				cmd_xprintf_P(pUSB,PSTR("RI=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RTS
			if (!strcmp_P( strupr(argv[2]), PSTR("RTS\0")) ) {
				pin = IO_read_RTS();
				cmd_xprintf_P(pUSB,PSTR("RTS=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}


			pv_snprintfP_ERR();
			return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode )
{
	// Como se usa para leer memoria sram del RTC, la impresion la hacemos en hex

bool retS;
uint8_t length = 0;
char *p;
uint8_t i;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(aux_buffer, '\0', sizeof(aux_buffer));
		retS = RTC_read( (uint32_t)(atol(argv[2])), &aux_buffer[0], (uint8_t)(atoi(argv[3])) );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			cmd_xprintf_P( pUSB, PSTR("\r\n\0"));
			for (i=0; i < atoi(argv[3]); i++ ) {
				cmd_xprintf_P( pUSB,PSTR("[0x%02x]"),aux_buffer[i]);
			}
			cmd_xprintf_P( pUSB,PSTR( "\r\n\0"));
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write rtc pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		retS = RTC_write( (uint32_t)(atol(argv[2])), argv[3], length );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}
}
//------------------------------------------------------------------------------------

