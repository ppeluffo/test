/*
 * drv_nvm_spx.c
 *
 *  Created on: 3 jul. 2018
 *      Author: pablo
 */


#include "drv_nvm_spx.h"

static void pv_drvNVM_EEPROM_WaitForNVM( void );
static void pv_drvNVM_EEPROM_FlushBuffer( void );

#define NVM_EXEC()	asm("push r30"      "\n\t"	\
			    "push r31"      "\n\t"	\
    			    "push r16"      "\n\t"	\
    			    "push r18"      "\n\t"	\
			    "ldi r30, 0xCB" "\n\t"	\
			    "ldi r31, 0x01" "\n\t"	\
			    "ldi r16, 0xD8" "\n\t"	\
			    "ldi r18, 0x01" "\n\t"	\
			    "out 0x34, r16" "\n\t"	\
			    "st Z, r18"	    "\n\t"	\
    			    "pop r18"       "\n\t"	\
			    "pop r16"       "\n\t"	\
			    "pop r31"       "\n\t"	\
			    "pop r30"       "\n\t"	\
			    )

//------------------------------------------------------------------------------------
void drvNVM_EEPROM_write_buffer(uint16_t address, const void *buf, uint16_t len)
{
	// Escribe un buffer en la internal EEprom de a 1 byte. No considero paginacion.
	// Utiliza una funcion que hace erase&write.

	if ( address >= EEPROM_SIZE) return;

	while (len > 0) {
		drvNVM_EEPROM_write_byte(address++, *(uint8_t*)buf);
        buf = (uint8_t*)buf + 1;
        len--;
    }
}
//------------------------------------------------------------------------------------
void drvNVM_EEPROM_write_byte(uint16_t address, uint8_t value)
{
	// Escribe de a 1 byte en la internal EEprom

	if ( address >= EEPROM_SIZE ) return;

    /*  Flush buffer to make sure no unintentional data is written and load
     *  the "Page Load" command into the command register.
     */
    pv_drvNVM_EEPROM_FlushBuffer();
    NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

    // Set address to write to
    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xFF;
    NVM.ADDR0 = address & 0xFF;

	/* Load data to write, which triggers the loading of EEPROM page buffer. */
	NVM.DATA0 = value;

	/*  Issue EEPROM Atomic Write (Erase&Write) command. Load command, write
	 *  the protection signature and execute command.
	 */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();

}
//------------------------------------------------------------------------------------
uint8_t drvNVM_EEPROM_ReadByte( uint16_t address )
{
	/* Wait until NVM is not busy. */
	pv_drvNVM_EEPROM_WaitForNVM();

	/* Set address to read from. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0xFF;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Read command. */
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;
	NVM_EXEC();

	return NVM.DATA0;
}
//------------------------------------------------------------------------------------
void drvNVM_EEPROM_read_buffer(uint16_t address, char *buf, uint16_t len)
{

uint8_t rb;

	if ( address >= EEPROM_SIZE) return;

	while (len--) {
		rb = drvNVM_EEPROM_ReadByte(address++);
		*buf++ = rb;
    }

}
//------------------------------------------------------------------------------------
void drvNVM_EEPROM_EraseAll( void )
{
	/* Wait until NVM is not busy. */
	pv_drvNVM_EEPROM_WaitForNVM();

	/* Issue EEPROM Erase All command. */
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
	NVM_EXEC();
}
//------------------------------------------------------------------------------------
uint8_t drvNVM_ReadSignatureByte(uint16_t Address)
{

	// Funcion que lee la memoria NVR, la calibration ROW de a una posicion ( de 16 bits )

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	//  __asm__ ("lpm \n  mov %0, r0 \n" : "=r" (Result) : "z" (Address) : "r0");
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}
//----------------------------------------------------------------------------------------
static void pv_drvNVM_EEPROM_FlushBuffer( void )
{
	// Flushea el eeprom page buffer.

	/* Wait until NVM is not busy. */
	pv_drvNVM_EEPROM_WaitForNVM();

	/* Flush EEPROM page buffer if necessary. */
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}
//------------------------------------------------------------------------------------
static void pv_drvNVM_EEPROM_WaitForNVM( void )
{
	/* Wait for any NVM access to finish, including EEPROM.
 	 *
 	 *  This function is blcoking and waits for any NVM access to finish,
 	 *  including EEPROM. Use this function before any EEPROM accesses,
 	 *  if you are not certain that any previous operations are finished yet,
 	 *  like an EEPROM write.
 	 */
	do {
		/* Block execution while waiting for the NVM to be ready. */
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}
//------------------------------------------------------------------------------------
