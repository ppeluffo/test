/*
 * l_analogCh.c
 *
 *  Created on: 3 jul. 2018
 *      Author: pablo
 */


#include "l_anCh.h"

//------------------------------------------------------------------------------------
void ACH_config( uint8_t ina_id, uint16_t conf_reg_value )
{
char res[3];

	res[0] = ( conf_reg_value & 0xFF00 ) >> 8;
	res[1] = ( conf_reg_value & 0x00FF );
	INA_write( ina_id, INA3231_CONF, res, 2 );
}
//------------------------------------------------------------------------------------
uint16_t ACH_read_channel( uint8_t channel_id )
{

uint8_t ina_reg = 0;
uint8_t ina_id = 0;
uint16_t an_raw_val;
char res[3];

	switch ( channel_id ) {
	case 0:
		ina_id = 0; ina_reg = INA3221_CH1_SHV;break;
	case 1:
		ina_id = 0; ina_reg = INA3221_CH2_SHV;break;
	case 2:
		ina_id = 0; ina_reg = INA3221_CH3_SHV;break;
	case 3:
		ina_id = 1; ina_reg = INA3221_CH2_SHV;break;
	case 4:
		ina_id = 1; ina_reg = INA3221_CH3_SHV;break;
	case 5:
		ina_id = 1; ina_reg = INA3221_CH1_BUSV;break;	// Battery
	default:
		return(-1);
	}

	// Leo el valor del INA.
	INA_read( ina_id, ina_reg, res ,2 );
	an_raw_val = 0;
	an_raw_val = ( res[0]<< 8 ) + res[1];
	an_raw_val = an_raw_val >> 3;

	return( an_raw_val );
}
//------------------------------------------------------------------------------------

