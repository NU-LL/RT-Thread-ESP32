/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

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

/*******************************************************************************
// Copyright (c) 2003-2015 Cadence Design Systems, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <string.h>
#include <xtensa/config/core.h>

#include "xtensa_rtos.h"

#include "esp32/rom/ets_sys.h"
#include "soc/cpu.h"

#include "FreeRTOS.h"
#include "task.h"

#include "esp_debug_helpers.h"
#include "esp_heap_caps.h"
#include "esp_private/crosscore_int.h"

#include "esp_intr_alloc.h"
#include "esp_log.h"


#include <rtthread.h>


/* Defined in portasm.h */
extern void _frxt_tick_timer_init(void);

/* Defined in xtensa_context.S */
extern void _xt_coproc_init(void);


#if CONFIG_FREERTOS_CORETIMER_0
    #define SYSTICK_INTR_ID (ETS_INTERNAL_TIMER0_INTR_SOURCE+ETS_INTERNAL_INTR_SOURCE_OFF)
#endif
#if CONFIG_FREERTOS_CORETIMER_1
    #define SYSTICK_INTR_ID (ETS_INTERNAL_TIMER1_INTR_SOURCE+ETS_INTERNAL_INTR_SOURCE_OFF)
#endif

_Static_assert(tskNO_AFFINITY == CONFIG_FREERTOS_NO_AFFINITY, "incorrect tskNO_AFFINITY value");

/*-----------------------------------------------------------*/

unsigned port_xSchedulerRunning[portNUM_PROCESSORS] = {0}; // Duplicate of inaccessible xSchedulerRunning; needed at startup to avoid counting nesting
unsigned port_interruptNesting[portNUM_PROCESSORS] = {0};  // Interrupt nesting level. Increased/decreased in portasm.c, _frxt_int_enter/_frxt_int_exit

/*
 * Stack initialization
 */
// 这部分代码已经挪到 libcpu/xtensa/esp32/cpuport.c

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the Xtensa port will get stopped.  If required simply
	disable the tick interrupt here. */
    rt_kprintf("error : vPortEndScheduler not implemented!!!\n\n");
}

/*-----------------------------------------------------------*/
// 开始调度
// FreeRTOS 中的代码 这里暂时放着不管
BaseType_t xPortStartScheduler( void )
{
	// // Interrupts are disabled at this point and stack contains PS with enabled interrupts when task context is restored

	// #if XCHAL_CP_NUM > 0
	// /* Initialize co-processor management for tasks. Leave CPENABLE alone. */
	// _xt_coproc_init();
	// #endif

	// /* Init the tick divisor value */
	// _xt_tick_divisor_init();

	// /* Setup the hardware to generate the tick. */
	// _frxt_tick_timer_init();

	// port_xSchedulerRunning[xPortGetCoreID()] = 1;

	// // Cannot be directly called from C; never returns
	// __asm__ volatile ("call0    _frxt_dispatch\n");

	// /* Should not get here. */

    rt_kprintf("error : xPortStartScheduler not implemented!!!\n\n");

	return pdTRUE;
}

/*-----------------------------------------------------------*/
// 定时器中断
BaseType_t xPortSysTickHandler( void )
{
    // rt_kprintf("tick:%d\n", rt_tick_get()+1);
    // rt_kprintf("tick:%d ", rt_tick_get()+1);
	BaseType_t ret;

	portbenchmarkIntLatency();
	traceISR_ENTER(SYSTICK_INTR_ID);
	ret = xTaskIncrementTick();
	if( ret != pdFALSE )
	{
		portYIELD_FROM_ISR();
	} else {
		traceISR_EXIT();
	}
	return ret;
}

// 切换 cpu 核心代码 这里直接为空
void vPortYieldOtherCore( BaseType_t coreid ) {
	// esp_crosscore_int_send_yield( coreid );
    rt_kprintf("error : vPortYieldOtherCore not implemented!!!\n\n");
}

/*-----------------------------------------------------------*/

/*
 * Used to set coprocessor area in stack. Current hack is to reuse MPU pointer for coprocessor area.
 */
// #if portUSING_MPU_WRAPPERS
// void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t usStackDepth )
// {
// 	#if XCHAL_CP_NUM > 0
// 	xMPUSettings->coproc_area = (StackType_t*)((((uint32_t)(pxBottomOfStack + usStackDepth - 1)) - XT_CP_SIZE ) & ~0xf);


// 	/* NOTE: we cannot initialize the coprocessor save area here because FreeRTOS is going to
//          * clear the stack area after we return. This is done in pxPortInitialiseStack().
// 	 */
// 	#endif
// }

// void vPortReleaseTaskMPUSettings( xMPU_SETTINGS *xMPUSettings )
// {
// 	/* If task has live floating point registers somewhere, release them */
// 	_xt_coproc_release( xMPUSettings->coproc_area );
// }

// #endif

/*
 * Returns true if the current core is in ISR context; low prio ISR, med prio ISR or timer tick ISR. High prio ISRs
 * aren't detected here, but they normally cannot call C code, so that should not be an issue anyway.
 */
BaseType_t xPortInIsrContext()
{
	unsigned int irqStatus;
	BaseType_t ret;
	irqStatus=portENTER_CRITICAL_NESTED();
    // 这里直接拿中断嵌套层数
    // 用于判断是否处于中断上下文中
    ret = (rt_interrupt_get_nest() != 0);
	// ret=(port_interruptNesting[xPortGetCoreID()] != 0);
	portEXIT_CRITICAL_NESTED(irqStatus);
	return ret;
}

/*
 * This function will be called in High prio ISRs. Returns true if the current core was in ISR context
 * before calling into high prio ISR context.
 */
BaseType_t IRAM_ATTR xPortInterruptedFromISRContext()
{
	// return (port_interruptNesting[xPortGetCoreID()] != 0);
    return (rt_interrupt_get_nest() != 0);
}

void vPortAssertIfInISR()
{
	configASSERT(xPortInIsrContext());
}

/*
 * For kernel use: Initialize a per-CPU mux. Mux will be initialized unlocked.
 */
// 多核 CPU 中使用 这里用单核直接不管
void vPortCPUInitializeMutex(portMUX_TYPE *mux) {

// #ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
// 	ets_printf("Initializing mux %p\n", mux);
// 	mux->lastLockedFn="(never locked)";
// 	mux->lastLockedLine=-1;
// #endif
// 	mux->owner=portMUX_FREE_VAL;
// 	mux->count=0;
    // rt_kprintf("error : vPortCPUInitializeMutex not implemented!!!\n\n");
}

uint32_t xPortGetTickRateHz(void) {
	return (uint32_t)configTICK_RATE_HZ;
}
