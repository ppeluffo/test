/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */
#include <spx.h>
#include "frtos_io.h"
#include "frtos_cmd.h"
#include "l_rtc.h"
#include "l_eeprom.h"
#include "l_ina.h"
#include "l_anCh.h"

#define WR_CMD 0
#define RD_CMD 1

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_rwEE(uint8_t cmd_mode );
static void pv_cmd_rwINA(uint8_t cmd_mode );
static void pv_cmd_rwACH(uint8_t cmd_mode );

RtcTimeType_t rtc;
char aux_buffer[32];

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

uint8_t c;

( void ) pvParameters;

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

	// loop
	for( ;; )
	{

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( sFRTOS_read( pUSB, (char *)&c, 1 ) == 1 ) {
			FRTOS_CMD_process(c);
		}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{
	cmd_xprintf_P(pUSB,PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	cmd_xprintf_P(pUSB,PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );

	// DlgId
	cmd_xprintf_P(pUSB,PSTR("dlgid: ??\r\n\0") );
	cmd_xprintf_P(pUSB,PSTR("size_t: %d\r\n\0"), sizeof(size_t) );
	cmd_xprintf_P(pUSB,PSTR("unsigned int: %d\r\n\0"), sizeof(unsigned int) );
	cmd_xprintf_P(pUSB,PSTR("int: %d\r\n\0"), sizeof( int) );
	cmd_xprintf_P(pUSB,PSTR("port BASE: %d\r\n\0"), sizeof( portBASE_TYPE) );
	cmd_xprintf_P(pUSB,PSTR("UBaseType_t: %d\r\n\0"), sizeof(UBaseType_t) );
	cmd_xprintf_P(pUSB,PSTR("uint8_t: %d\r\n\0"), sizeof( uint8_t ) );


}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

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
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) ) {
		pv_cmd_rwINA(WR_CMD);
		return;
	}

	// ACH
	// write ach {id} conf128
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0")) ) {
		pv_cmd_rwACH(WR_CMD);
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

	// RTC
	// read rtc
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("INA\0")) ) {
		pv_cmd_rwINA(RD_CMD);
		return;
	}

	// ACH
	// read aCh {ch}, bat
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0")) ) {
		pv_cmd_rwACH(RD_CMD);
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

	FRTOS_CMD_makeArgv();


}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr( (char *)argv[1]), PSTR("WRITE\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-write\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  rtc YYMMDDhhmm\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  ee {pos} {string}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  ina {id} conf {value}, sens12V {on|off}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  ach {id} conf128 \r\n\0"));
		return;

	// HELP READ
	} else if (!strcmp_P( strupr( (char *)argv[1]), PSTR("READ\0"))) {
		cmd_xprintf_P(pUSB,PSTR("-read\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  rtc, frame\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  ee {pos} {lenght}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		cmd_xprintf_P(pUSB,PSTR("  aCh {ch}, bat \r\n\0"));
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

	pv_snprintfP_OK();
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


