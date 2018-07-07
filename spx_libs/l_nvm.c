/*
 * l_nvm.c
 *
 *  Created on: 4 jul. 2018
 *      Author: pablo
 */

#include "l_nvm.h"

bool signature = false;
char signature_str[12];

//----------------------------------------------------------------------------
int8_t NVMEE_read( uint32_t rdAddress, char *data, uint16_t length )
{
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

size_t xReturn = 0U;
uint16_t device_address;
uint8_t xBytes = 0;

	sFRTOS_ioctl (pNVM,ioctlOBTAIN_BUS_SEMPH, NULL);

	device_address = (uint16_t)(rdAddress);
	sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &device_address);

	//  3) Por ultimo leemos. No controlo fronteras.
	xBytes = length;
	xReturn = sFRTOS_read( pNVM, data, xBytes);
	if (xReturn != xBytes ) {
		xReturn = -1;
	}

	sFRTOS_ioctl (pNVM,ioctlRELEASE_BUS_SEMPH, NULL);

	return(xReturn);

}
//----------------------------------------------------------------------------
int8_t NVMEE_write( uint32_t wrAddress, char *data, uint16_t length )
{

size_t xReturn = 0U;
uint16_t device_address;
uint8_t xBytes = 0;

	sFRTOS_ioctl (pNVM,ioctlOBTAIN_BUS_SEMPH, NULL);

	device_address = (uint16_t)(wrAddress);
	sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &device_address);

	//  3) Por ultimo escribimos. No controlo fronteras.
	xBytes = length;
	xReturn = sFRTOS_write(pNVM, data, xBytes);
	if (xReturn != xBytes ) {
		xReturn = -1;
	}

	sFRTOS_ioctl (pNVM,ioctlRELEASE_BUS_SEMPH, NULL);

	return(xReturn);

}
//----------------------------------------------------------------------------
char *NVMEE_readID(void)
{
uint8_t address;

	if ( !signature ) {
		memset(signature_str,'\0', sizeof(signature_str));

		address = LOTNUM0; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[0] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = LOTNUM1; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[1] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = LOTNUM2; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[2] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = LOTNUM3; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[3] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = LOTNUM4; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[4] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = LOTNUM5; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[5] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
//		address = WAFNUM;  sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[6] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		signature_str[6] = 'A';
//		address = COORDX0; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[7] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		signature_str[7] = 'B';
		address = COORDX1; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[8] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = COORDY0; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[9] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );
		address = COORDX1; sFRTOS_ioctl(pNVM, ioctl_NVM_SET_BYTEADDRESS, &address ); signature_str[10] = sFRTOS_ioctl(pNVM, ioctl_NVM_READ_SIGNATURE, 0 );

		signature = true;
	}

	uint8_t i;
	for (i=0; i < 10; i++) {
		sFRTOS_write(pUSB, &signature_str[i],1);
	}

	return(&signature_str[0]);

}
//----------------------------------------------------------------------------
