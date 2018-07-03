/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPXR1_H_
#define SRC_SPXR1_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include <inttypes.h>

#include "TC_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "frtos_io.h"
#include "l_iopines.h"
#include "l_i2c.h"
#include "l_rtc.h"
#include "l_ina.h"
#include "l_anCh.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "0.0.6"
#define SPX_FW_DATE "@ 20180703"

#define SPX_HW_MODELO "spx HW:xmega256A3B R1.0"
#define SPX_FTROS_VERSION "FW:test9 FRTOS10"

// El datalogger tiene 6 canales fisicos pero 5 disponibles
// ya que uno esta para monitorear la bateria.
#define NRO_ANALOG_CHANNELS		5
#define NRO_DIGITAL_CHANNELS	4

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32

#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )

TaskHandle_t xHandle_tkCtl,xHandle_tkCmd;

//------------------------------------------------------------------------------------

bool startTask;
//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

// Utils
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
void initMCU(void);
void xprintf_P_init(void);
int xprintf_P( int fd, PGM_P fmt, ...);
int cmd_xprintf_P( int fd, PGM_P fmt, ...);


#endif /* SRC_SPXR1_H_ */
