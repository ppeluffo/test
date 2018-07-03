/*
 * l_analogCh.h
 *
 *  Created on: 3 jul. 2018
 *      Author: pablo
 */

#ifndef SPX_LIBS_L_ANCH_H_
#define SPX_LIBS_L_ANCH_H_

#include "frtos_io.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "l_ina.h"

//------------------------------------------------------------------------------------

void ACH_config( uint8_t ina_id, uint16_t conf_reg_value );
uint16_t ACH_read_channel( uint8_t channel_id );

#define ACH_read_battery()	ACH_read_channel(5)
#define ACH_config_avg128(ina_id)	ACH_config(ina_id, CONF_INAS_AVG128 )
#define ACH_config_sleep(ina_id)	ACH_config(ina_id, CONF_INAS_SLEEP )

#endif /* SPX_LIBS_L_ANCH_H_ */
