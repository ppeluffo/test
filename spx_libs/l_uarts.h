/*
 * uarts_sp5K.h
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_UARTS_H_
#define SRC_SPX_LIBS_L_UARTS_H_


void CMD_write( const void *pvBuffer, const size_t xBytes );
size_t CMD_read( void *pvBuffer, const size_t xBytes );
void CMD_writeChar (unsigned char c);
size_t uXBEE_read(  void *pvBuffer, const size_t xBytes );

#endif /* SRC_SPX_LIBS_L_UARTS_H_ */
