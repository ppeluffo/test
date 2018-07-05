/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//#include "time.h" 		// Needed for system_tick();


#include "avr_compiler.h"
#include "FreeRTOSConfig.h"
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "TC_driver.h"
#include "pmic_driver.h"

#include <avr/sleep.h>
#include "l_iopines.h"

#define __AVR_XMEGA_256A3B__

#define FOSC_RTC32		1024
static volatile bool	countingCompleted;
static void prvSleepExactTime( portTickType xExpectedIdleTime );

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the AVR port.
 *----------------------------------------------------------*/

/* Start tasks with interrupts enabled. */
#define portFLAGS_INT_ENABLED				( (StackType_t) 0x80 )

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;

/* actual number of ticks per second, after configuration. Not for RTC, which has 1 tick/second. */
TickType_t portTickRateHz;

/* remaining ticks in each second, decremented to enable the system_tick. Not for RTC, which has 1 tick/second. */
volatile TickType_t ticksRemainingInSec;

/*-----------------------------------------------------------*/
/*
 * Perform hardware setup to enable ticks from timer compare match A.
 */
static void prvSetupTimerInterrupt( void );

/*-----------------------------------------------------------*/

/*
 * Macro to save all the general purpose registers, the save the stack pointer
 * into the TCB.
 *
 * The first thing we do is save the flags then disable interrupts.  This is to
 * guard our stack against having a context switch interrupt after we have already
 * pushed the registers onto the stack - causing the 32 registers to be on the
 * stack twice.
 *
 * r1 is set to zero as the compiler expects it to be thus, however some
 * of the math routines make use of R1.
 *
 * The interrupts will have been disabled during the call to portSAVE_CONTEXT()
 * so we need not worry about reading/writing to the stack pointer.
 */
#if defined(__AVR_XMEGA_256A3B__)
/* 3-Byte PC Save */
#define portSAVE_CONTEXT()                                                      \
        __asm__ __volatile__ (  "push   r0                                      \n\t"   \
                                "in		r0, __SREG__                    \n\t"   \
                                "cli                                            \n\t"   \
                                "push	r0                                      \n\t"   \
                                "in     r0, 0x3b                                \n\t"   \
                                "push   r0                                      \n\t"   \
                                "in     r0, 0x3c                                \n\t"   \
                                "push   r0                                      \n\t"   \
                                "push   r1                                      \n\t"   \
                                "clr    r1                                      \n\t"   \
                                "push   r2                                      \n\t"   \
                                "push   r3                                      \n\t"   \
                                "push   r4                                      \n\t"   \
                                "push   r5                                      \n\t"   \
                                "push   r6                                      \n\t"   \
                                "push   r7                                      \n\t"   \
                                "push   r8                                      \n\t"   \
                                "push   r9                                      \n\t"   \
                                "push   r10                                     \n\t"   \
                                "push   r11                                     \n\t"   \
                                "push   r12                                     \n\t"   \
                                "push   r13                                     \n\t"   \
                                "push   r14                                     \n\t"   \
                                "push   r15                                     \n\t"   \
                                "push   r16                                     \n\t"   \
                                "push   r17                                     \n\t"   \
                                "push   r18                                     \n\t"   \
                                "push   r19                                     \n\t"   \
                                "push   r20                                     \n\t"   \
                                "push   r21                                     \n\t"   \
                                "push   r22                                     \n\t"   \
                                "push   r23                                     \n\t"   \
                                "push   r24                                     \n\t"   \
                                "push   r25                                     \n\t"   \
                                "push   r26                                     \n\t"   \
                                "push   r27                                     \n\t"   \
                                "push   r28                                     \n\t"   \
                                "push   r29                                     \n\t"   \
                                "push   r30                                     \n\t"   \
                                "push   r31                                     \n\t"   \
                                "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "in     r0, 0x3d                                \n\t"   \
                                "st     x+, r0                                  \n\t"   \
                                "in     r0, 0x3e                                \n\t"   \
                                "st     x+, r0                                  \n\t"   \
                             );
#else
/* 2-Byte PC Save */
#define portSAVE_CONTEXT()                                                              \
        __asm__ __volatile__ (  "push   r0                                      \n\t"   \
                                "in     r0, __SREG__                            \n\t"   \
                                "cli                                            \n\t"   \
                                "push   r0                                      \n\t"   \
                                "push   r1                                      \n\t"   \
                                "clr    r1                                      \n\t"   \
                                "push   r2                                      \n\t"   \
                                "push   r3                                      \n\t"   \
                                "push   r4                                      \n\t"   \
                                "push   r5                                      \n\t"   \
                                "push   r6                                      \n\t"   \
                                "push   r7                                      \n\t"   \
                                "push   r8                                      \n\t"   \
                                "push   r9                                      \n\t"   \
                                "push   r10                                     \n\t"   \
                                "push   r11                                     \n\t"   \
                                "push   r12                                     \n\t"   \
                                "push   r13                                     \n\t"   \
                                "push   r14                                     \n\t"   \
                                "push   r15                                     \n\t"   \
                                "push   r16                                     \n\t"   \
                                "push   r17                                     \n\t"   \
                                "push   r18                                     \n\t"   \
                                "push   r19                                     \n\t"   \
                                "push   r20                                     \n\t"   \
                                "push   r21                                     \n\t"   \
                                "push   r22                                     \n\t"   \
                                "push   r23                                     \n\t"   \
                                "push   r24                                     \n\t"   \
                                "push   r25                                     \n\t"   \
                                "push   r26                                     \n\t"   \
                                "push   r27                                     \n\t"   \
                                "push   r28                                     \n\t"   \
                                "push   r29                                     \n\t"   \
                                "push   r30                                     \n\t"   \
                                "push   r31                                     \n\t"   \
                                "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "in     r0, 0x3d                                \n\t"   \
                                "st     x+, r0                                  \n\t"   \
                                "in     r0, 0x3e                                \n\t"   \
                                "st     x+, r0                                  \n\t"   \
                             );
#endif

/*
 * Opposite to portSAVE_CONTEXT().  Interrupts will have been disabled during
 * the context save so we can write to the stack pointer.
 */
#if defined(__AVR_XMEGA_256A3B__)
/* 3-Byte PC Restore */
#define portRESTORE_CONTEXT()                                                           \
        __asm__ __volatile__ (  "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "ld     r28, x+                                 \n\t"   \
                                "out    __SP_L__, r28                           \n\t"   \
                                "ld     r29, x+                                 \n\t"   \
                                "out    __SP_H__, r29                           \n\t"   \
                                "pop    r31                                     \n\t"   \
                                "pop    r30                                     \n\t"   \
                                "pop    r29                                     \n\t"   \
                                "pop    r28                                     \n\t"   \
                                "pop    r27                                     \n\t"   \
                                "pop    r26                                     \n\t"   \
                                "pop    r25                                     \n\t"   \
                                "pop    r24                                     \n\t"   \
                                "pop    r23                                     \n\t"   \
                                "pop    r22                                     \n\t"   \
                                "pop    r21                                     \n\t"   \
                                "pop    r20                                     \n\t"   \
                                "pop    r19                                     \n\t"   \
                                "pop    r18                                     \n\t"   \
                                "pop    r17                                     \n\t"   \
                                "pop    r16                                     \n\t"   \
                                "pop    r15                                     \n\t"   \
                                "pop    r14                                     \n\t"   \
                                "pop    r13                                     \n\t"   \
                                "pop    r12                                     \n\t"   \
                                "pop    r11                                     \n\t"   \
                                "pop    r10                                     \n\t"   \
                                "pop    r9                                      \n\t"   \
                                "pop    r8                                      \n\t"   \
                                "pop    r7                                      \n\t"   \
                                "pop    r6                                      \n\t"   \
                                "pop    r5                                      \n\t"   \
                                "pop    r4                                      \n\t"   \
                                "pop    r3                                      \n\t"   \
                                "pop    r2                                      \n\t"   \
                                "pop    r1                                      \n\t"   \
                                "pop    r0                                      \n\t"   \
                                "out    0x3c, r0                                \n\t"   \
                                "pop    r0                                      \n\t"   \
                                "out    0x3b, r0                                \n\t"   \
                                "pop    r0                                      \n\t"   \
                                "out    __SREG__, r0                            \n\t"   \
                                "pop    r0                                      \n\t"   \
                             );
#else
/* 2-Byte PC Restore */
#define portRESTORE_CONTEXT()                                                           \
        __asm__ __volatile__ (  "lds    r26, pxCurrentTCB                       \n\t"   \
                                "lds    r27, pxCurrentTCB + 1                   \n\t"   \
                                "ld     r28, x+                                 \n\t"   \
                                "out    __SP_L__, r28                           \n\t"   \
                                "ld     r29, x+                                 \n\t"   \
                                "out    __SP_H__, r29                           \n\t"   \
                                "pop    r31                                     \n\t"   \
                                "pop    r30                                     \n\t"   \
                                "pop    r29                                     \n\t"   \
                                "pop    r28                                     \n\t"   \
                                "pop    r27                                     \n\t"   \
                                "pop    r26                                     \n\t"   \
                                "pop    r25                                     \n\t"   \
                                "pop    r24                                     \n\t"   \
                                "pop    r23                                     \n\t"   \
                                "pop    r22                                     \n\t"   \
                                "pop    r21                                     \n\t"   \
                                "pop    r20                                     \n\t"   \
                                "pop    r19                                     \n\t"   \
                                "pop    r18                                     \n\t"   \
                                "pop    r17                                     \n\t"   \
                                "pop    r16                                     \n\t"   \
                                "pop    r15                                     \n\t"   \
                                "pop    r14                                     \n\t"   \
                                "pop    r13                                     \n\t"   \
                                "pop    r12                                     \n\t"   \
                                "pop    r11                                     \n\t"   \
                                "pop    r10                                     \n\t"   \
                                "pop    r9                                      \n\t"   \
                                "pop    r8                                      \n\t"   \
                                "pop    r7                                      \n\t"   \
                                "pop    r6                                      \n\t"   \
                                "pop    r5                                      \n\t"   \
                                "pop    r4                                      \n\t"   \
                                "pop    r3                                      \n\t"   \
                                "pop    r2                                      \n\t"   \
                                "pop    r1                                      \n\t"   \
                                "pop    r0                                      \n\t"   \
                                "out    __SREG__, r0                            \n\t"   \
                                "pop    r0                                      \n\t"   \
                             );
#endif
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
uint16_t usAddress;

	/* Place a few bytes of known values on the bottom of the stack.
	This is just useful for debugging. */

	*pxTopOfStack = 0x11;
	pxTopOfStack--;
	*pxTopOfStack = 0x22;
	pxTopOfStack--;
	*pxTopOfStack = 0x33;
	pxTopOfStack--;

	/* Simulate how the stack would look after a call to vPortYield() generated by
	the compiler. */

	/*lint -e950 -e611 -e923 Lint doesn't like this much - but nothing I can do about it. */

	/* The start of the task code will be popped off the stack last, so place
	it on first. */

#if defined(__AVR_XMEGA_256A3B__)
	/* The AVR ATmega2560/ATmega2561 have 256KBytes of program memory and a 17-bit
	 * program counter.  When a code address is stored on the stack, it takes 3 bytes
	 * instead of 2 for the other ATmega* chips.
	 *
	 * Store 0 as the top byte since we force all task routines to the bottom 128K
	 * of flash. We do this by using the .lowtext label in the linker script.
	 *
	 * In order to do this properly, we would need to get a full 3-byte pointer to
	 * pxCode.  That requires a change to GCC.  Not likely to happen any time soon.
	 */
	usAddress = ( uint16_t ) pxCode;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	*pxTopOfStack = 0;
	pxTopOfStack--;
#else
	usAddress = ( uint16_t ) pxCode;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;
#endif

	/* Next simulate the stack as if after a call to portSAVE_CONTEXT().
	portSAVE_CONTEXT places the flags on the stack immediately after r0
	to ensure the interrupts get disabled as soon as possible, and so ensuring
	the stack use is minimal should a context switch interrupt occur. */
	*pxTopOfStack = ( StackType_t ) 0x00;	/* R0 */
	pxTopOfStack--;
	*pxTopOfStack = portFLAGS_INT_ENABLED;
	pxTopOfStack--;

#if defined(__AVR_XMEGA_256A3B__)

	/* If we have an ATmega256x, we are also saving the RAMPZ and EIND registers.
	 * We should default those to 0.
	 */
	*pxTopOfStack = ( StackType_t ) 0x00;	/* EIND */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x00;	/* RAMPZ */
	pxTopOfStack--;

#endif

	/* Now the remaining registers.   The compiler expects R1 to be 0. */
	*pxTopOfStack = ( StackType_t ) 0x00;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x13;	/* R13 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x14;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x15;	/* R15 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x16;	/* R16 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x17;	/* R17 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x18;	/* R18 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x19;	/* R19 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x20;	/* R20 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x21;	/* R21 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x22;	/* R22 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x23;	/* R23 */
	pxTopOfStack--;

	/* Place the parameter on the stack in the expected location. */
	usAddress = ( uint16_t ) pvParameters;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0x26;	/* R26 X */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x27;	/* R27 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x28;	/* R28 Y */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x29;	/* R29 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x30;	/* R30 Z */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x031;	/* R31 */
	pxTopOfStack--;

	/*lint +e950 +e611 +e923 */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{

	/* Setup the relevant timer hardware to generate the tick. */
	prvSetupTimerInterrupt();

	/* Restore the context of the first task that is going to run. */
	portRESTORE_CONTEXT();

	/* Simulate a function call end as generated by the compiler.  We will now
	jump to the start of the task the context of which we have just restored. */
	__asm__ __volatile__ ( "ret" );

	/* Should not get here. */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the AVR port will get stopped.  If required simply
	disable the tick interrupt here. */

}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield( void ) __attribute__ ( ( hot, flatten, naked ) );
void vPortYield( void )
{
	portSAVE_CONTEXT();
	vTaskSwitchContext();
	portRESTORE_CONTEXT();

	__asm__ __volatile__ ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick.  This must be identical to
 * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void ) __attribute__ ( ( hot, flatten, naked ) );
void vPortYieldFromTick( void )
{
	portSAVE_CONTEXT();

	sleep_disable();		//	 reset the sleep_mode() faster than sleep_disable();

	if( xTaskIncrementTick() != pdFALSE )
	{
		vTaskSwitchContext();

	}

	portRESTORE_CONTEXT();

	__asm__ __volatile__ ( "ret" );
}
/*-----------------------------------------------------------*/
/*
 * Setup timer 0 compare match A to generate a tick interrupt.
 */

static void prvSetupTimerInterrupt(void) {
    //Use TCC0 as a tick counter. If this is to be changed, change ISR as well
    TC0_t * tickTimer = &TCC0;

    //select the clock source and pre-scale by 64
    // 32Mhz -> 5Mhz
    //  8Mhz -> 125Khz
    //  2Mhz -> 31.250Khz
    TC0_ConfigClockSource(tickTimer, TC_CLKSEL_DIV64_gc);

    //set period of counter
    tickTimer->PER = configCPU_CLOCK_HZ / configTICK_RATE_HZ / 64 - 1;

    //enable interrupt and set low level
    TC0_SetOverflowIntLevel(tickTimer, TC_OVFINTLVL_LO_gc);
    //enable low level interrupts
    PMIC_EnableLowLevel();
}
/*-----------------------------------------------------------*/
#if configUSE_PREEMPTION == 1

/*
 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
 * the context is saved at the start of vPortYieldFromTick().  The tick
 * count is incremented after the context is saved.
 */
// ----------------------------------------------
//MODIFICADO PARA USAR LA ISR DEL RTC !!!!!!!!!
// ----------------------------------------------
ISR (TCC0_OVF_vect, ISR_NAKED) {
    /*
     * Context switch function used by the tick.  This must be identical to
     * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
     * difference from vPortYield() is the tick count is incremented as the
     * call comes from the tick ISR.
     */
//    portSAVE_CONTEXT();
//    vTaskIncrementTick();
//    vTaskSwitchContext();
//   portRESTORE_CONTEXT();
	PORTA.OUTTGL = 0x04;	// Toggle A2
	vPortYieldFromTick();
    asm volatile ( "reti" );
}

#else

/*
 * Tick ISR for the cooperative scheduler.  All this does is increment the
 * tick count.  We don't need to switch context, this can only be done by
 * manual calls to taskYIELD();
 */
ISR (TCC0_OVF_vect, ISR_NAKED)
{
    vTaskIncrementTick();
}
#endif

//-------------------------------------------------------------------------------------
//  Tickless sleeping mode.

#if configUSE_TICKLESS_IDLE == 2
//-------------------------------------------------------------------------------------
static void prvSleep()
{
	/* Re-enabling interrupts to awake and go to sleep*/
	sleep_enable();
	sei();
	//sleep_cpu();
	sleep_mode();

	/* Sleeps here until awaken, then continues */
	sleep_disable();
	cli();
}
//-------------------------------------------------------------------------------------
// Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP().
void vApplicationSleep( TickType_t xExpectedIdleTime );

void vApplicationSleep( TickType_t xExpectedIdleTime )
{
	// Utilizo el RTC16 bits para contar el tiempo de sleep.

eSleepModeStatus eSleepStatus;

	// Paso1: Leer el 'tiempo actual'
	// Como el contador del RTC16 lo inicializo en 0, este es el tiempo inicial.
	// ulLowPowerTimeBeforeSleep = ulGetExternalTime();

	// Paso2: Detener la generacion de ticks.
	portENTER_CRITICAL();
    //prvStopTickInterruptTimer();

    // Enter a critical section that will not effect interrupts bringing the MCU
    // out of sleep mode.
    //disable_interrupts();

    // Paso3: Ensure it is still ok to enter the sleep mode.
    eSleepStatus = eTaskConfirmSleepModeStatus();

    if( eSleepStatus == eNoTasksWaitingTimeout ) {
        /* It is not necessary to configure an interrupt to bring the
        microcontroller out of its low power state at a fixed time in the
        future. */
    	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        prvSleep();
        portEXIT_CRITICAL();

    } else  if( eSleepStatus == eAbortSleep ) {
        /* A task has been moved out of the Blocked state since this macro was
        executed, or a context swith is being held pending.  Do not enter a
        sleep state.  Restart the tick and exit the critical section. */
    	portEXIT_CRITICAL();
        //prvStartTickInterruptTimer();
        //enable_interrupts();
    } else {
    	/* Configure an interrupt to bring the microcontroller out of its low
    	   power state at the time the kernel next needs to execute.  The
    	   interrupt must be generated from a source that remains operational
    	   when the microcontroller is in a low power state. */

    	portEXIT_CRITICAL();
    	prvSleepExactTime(xExpectedIdleTime);

    }

}
//-------------------------------------------------------------------------------------
static void prvSleepExactTime( portTickType xExpectedIdleTime )
{

portTickType tick_to_sleep, completedTicks;
uint32_t rtc_counter;
//uint16_t max_idle_time;

	// En esta funcion debo:
	// 1- Configurar el RTC16 para interrumpir luego de xExpectedIdleTime
	// 2- Sleep
	// 3- Al despertar, ajustar el tick a los ticks dormidos reales.

	// El RTC lo debo programar para dormir TICKS ( 10ms ) o sea que este va a ser el error
	// al despertar ( +/- 1 tick).
	// Por otro lado, el kernel determina cuanto puede dormir y este es el valor que pasa en xExpectedIdleTime.
	// Nosotros no queremos dormir mas de 1s por lo tanto calculo el valor del RTC y si supera 1s, lo limito.

	// Paso 1:
	// En MAIN ya configure el RTC con el osc externo a 32Khz sin prescaler.
	// El periodo son 0.03 ms por lo que para contar 10ms debo contar hasta 327 y para
	// contar 1s debo contar hasta 32768.
	// El contador es de 16bits o sea cuenta hasta 65536
	// Lo configuro para contar hasta xExpectedIdleTime.
	// Nunca cuento mas de 1500 ms

	// No se aplica cuando uso el RTC32.


	tick_to_sleep = xExpectedIdleTime;
//	max_idle_time = 1.5 * configTICK_RATE_HZ;

//	if ( tick_to_sleep > max_idle_time ) {
//		tick_to_sleep = max_idle_time;
//	}
	rtc_counter = tick_to_sleep * FOSC_RTC32 / configTICK_RATE_HZ - 1;

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	RTC32.PER = rtc_counter;
	RTC32.CNT = 0;
	RTC32.COMP = 0;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// Paso2: habilito las interrupciones y sleep.
	portENTER_CRITICAL();
	countingCompleted = false;

//	IO_clr_TICK();
	prvSleep();
//	IO_set_TICK();

	// Paso3: Al despertar actualizo los ticks.
	// Con countingComplete determino si complete el periodo de sleep o alguna
	// interrupcion me desperto antes.
	// Detengo el timer.
	RTC32.CTRL = 0x00;
	portEXIT_CRITICAL();

	if (countingCompleted) {
		// Durmio todo el periodo.
		completedTicks = xExpectedIdleTime;
	} else {
		// Algo lo desperto. Durmio de menos.
		// Como la ISR no borro el contador, vemos cuanto durmio y lo convierto a ticks.
		// Agrego 1 para redondear.
		completedTicks = ( RTC32.CNT * configTICK_RATE_HZ / FOSC_RTC32 ) +1;
	}

	vTaskStepTick(completedTicks);

}
//-------------------------------------------------------------------------------------
// interrupt routine RTC32 overflow */
ISR(RTC32_OVF_vect)
{
//	PORTA.OUTTGL = 0x04;	// Toggle A2
	countingCompleted = true;
}
//-------------------------------------------------------------------------------------
#endif /* configUSE_TICKLESS_IDLE == 2 */

