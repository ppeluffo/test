/*
 * l_nvm.h
 *
 *  Created on: 4 jul. 2018
 *      Author: pablo
 */

#ifndef SPX_LIBS_L_NVM_H_
#define SPX_LIBS_L_NVM_H_

#include "frtos_io.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

int8_t NVMEE_read( uint32_t rdAddress, char *data, uint8_t length );
int8_t NVMEE_write( uint32_t wrAddress, char *data, uint8_t length );
char *NVMEE_readID(void);

#endif /* SPX_LIBS_L_NVM_H_ */
