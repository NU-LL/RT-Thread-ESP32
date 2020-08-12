#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"

volatile rt_ubase_t  rt_interrupt_from_thread = 0;
volatile rt_ubase_t  rt_interrupt_to_thread   = 0;
volatile rt_uint32_t rt_thread_switch_interrupt_flag = 0;


struct rt_hw_stack_frame
{
    rt_ubase_t epc;        /* epc - epc    - program counter                     */
    rt_ubase_t ra;         /* x1  - ra     - return address for jumps            */
    rt_ubase_t mstatus;    /*              - machine status register             */
    rt_ubase_t gp;         /* x3  - gp     - global pointer                      */
    rt_ubase_t tp;         /* x4  - tp     - thread pointer                      */
    rt_ubase_t t0;         /* x5  - t0     - temporary register 0                */
    rt_ubase_t t1;         /* x6  - t1     - temporary register 1                */
    rt_ubase_t t2;         /* x7  - t2     - temporary register 2                */
    rt_ubase_t s0_fp;      /* x8  - s0/fp  - saved register 0 or frame pointer   */
    rt_ubase_t s1;         /* x9  - s1     - saved register 1                    */
    rt_ubase_t a0;         /* x10 - a0     - return value or function argument 0 */
    rt_ubase_t a1;         /* x11 - a1     - return value or function argument 1 */
    rt_ubase_t a2;         /* x12 - a2     - function argument 2                 */
    rt_ubase_t a3;         /* x13 - a3     - function argument 3                 */
    rt_ubase_t a4;         /* x14 - a4     - function argument 4                 */
    rt_ubase_t a5;         /* x15 - a5     - function argument 5                 */
    rt_ubase_t a6;         /* x16 - a6     - function argument 6                 */
    rt_ubase_t a7;         /* x17 - s7     - function argument 7                 */
    rt_ubase_t s2;         /* x18 - s2     - saved register 2                    */
    rt_ubase_t s3;         /* x19 - s3     - saved register 3                    */
    rt_ubase_t s4;         /* x20 - s4     - saved register 4                    */
    rt_ubase_t s5;         /* x21 - s5     - saved register 5                    */
    rt_ubase_t s6;         /* x22 - s6     - saved register 6                    */
    rt_ubase_t s7;         /* x23 - s7     - saved register 7                    */
    rt_ubase_t s8;         /* x24 - s8     - saved register 8                    */
    rt_ubase_t s9;         /* x25 - s9     - saved register 9                    */
    rt_ubase_t s10;        /* x26 - s10    - saved register 10                   */
    rt_ubase_t s11;        /* x27 - s11    - saved register 11                   */
    rt_ubase_t t3;         /* x28 - t3     - temporary register 3                */
    rt_ubase_t t4;         /* x29 - t4     - temporary register 4                */
    rt_ubase_t t5;         /* x30 - t5     - temporary register 5                */
    rt_ubase_t t6;         /* x31 - t6     - temporary register 6                */
};

/**
 * This function will initialize thread stack
 *
 * @param tentry the entry of thread
 * @param parameter the parameter of entry
 * @param stack_addr the beginning stack address
 * @param texit the function will be called when thread exit
 *
 * @return stack address
 */
// rt_uint8_t *rt_hw_stack_init(void       *tentry,
//                              void       *parameter,
//                              rt_uint8_t *stack_addr,
//                              void       *texit)
// {
//     struct rt_hw_stack_frame *frame;
//     rt_uint8_t         *stk;
//     int                i;

//     stk  = stack_addr + sizeof(rt_ubase_t);
//     stk  = (rt_uint8_t *)RT_ALIGN_DOWN((rt_ubase_t)stk, REGBYTES);
//     stk -= sizeof(struct rt_hw_stack_frame);

//     frame = (struct rt_hw_stack_frame *)stk;

//     for (i = 0; i < sizeof(struct rt_hw_stack_frame) / sizeof(rt_ubase_t); i++)
//     {
//         ((rt_ubase_t *)frame)[i] = 0xdeadbeef;
//     }

//     frame->ra      = (rt_ubase_t)texit;
//     frame->a0      = (rt_ubase_t)parameter;
//     frame->epc     = (rt_ubase_t)tentry;

//     /* force to machine mode(MPP=11) and set MPIE to 1 */
//     frame->mstatus = 0x00007880;

//     return stk;
// }

/*
 * #ifdef RT_USING_SMP
 * void rt_hw_context_switch_interrupt(void *context, rt_ubase_t from, rt_ubase_t to, struct rt_thread *to_thread);
 * #else
 * void rt_hw_context_switch_interrupt(rt_ubase_t from, rt_ubase_t to);
 * #endif
 */
// void rt_hw_context_switch_interrupt(rt_ubase_t from, rt_ubase_t to)
// {
//     if (rt_thread_switch_interrupt_flag == 0)
//         rt_interrupt_from_thread = from;

//     rt_interrupt_to_thread = to;
//     rt_thread_switch_interrupt_flag = 1;

//     return ;
// }





/** shutdown CPU */
void rt_hw_cpu_shutdown()
{
    rt_uint32_t level;
    rt_kprintf("shutdown...\n");

    level = rt_hw_interrupt_disable();
    while (level)
    {
        RT_ASSERT(0);
    }
}












#include <stdlib.h>
#include <string.h>
#include <xtensa/config/core.h>


// #include "esp32/rom/ets_sys.h"
// #include "soc/cpu.h"

// // #include "FreeRTOS.h"
// // #include "task.h"

// #include "esp_debug_helpers.h"
// #include "esp_heap_caps.h"
// #include "esp_private/crosscore_int.h"

// #include "esp_intr_alloc.h"
// #include "esp_log.h"







// #include "xtensa_rtos.h"

#include "esp32/rom/ets_sys.h"
#include "soc/cpu.h"

// #include "freertos/portmacro.h"
// #include "FreeRTOS.h"
// #include "task.h"

#include "esp_debug_helpers.h"
#include "esp_private/crosscore_int.h"

#include "esp_intr_alloc.h"
#include "esp_log.h"




/* Align a value up to nearest n-byte boundary, where n is a power of 2. */
#define ALIGNUP(n, val) (((val) + (n)-1) & -(n))

typedef rt_uint8_t			StackType_t;
typedef unsigned int	    UBaseType_t;
typedef void (*TaskFunction_t)( void * );

extern struct rt_thread *rt_current_thread;



// User exception dispatcher when exiting
void _xt_user_exit(void);

#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
// Wrapper to allow task functions to return (increases stack overhead by 16 bytes)
static void vPortTaskWrapper(TaskFunction_t pxCode, void *pvParameters)
{
	pxCode(pvParameters);
	//FreeRTOS tasks should not return. Log the task name and abort.
	// char * pcTaskName = pcTaskGetTaskName(NULL);

    char * pcTaskName = rt_current_thread->name;

	ESP_LOGE("FreeRTOS", "FreeRTOS Task \"%s\" should not return, Aborting now!", pcTaskName);
	abort();
}
#endif






/**
 * This function will initialize thread stack
 *
 * @param tentry the entry of thread
 * @param parameter the parameter of entry
 * @param stack_addr the beginning stack address
 * @param texit the function will be called when thread exit
 *
 * @return stack address
 */
rt_uint8_t *rt_hw_stack_init(void       *tentry,
                             void       *parameter,
                             rt_uint8_t *stack_addr,
                             void       *texit) //线程退出地址，即rt_thread_exit，暂时未考虑
// rt_uint8_t *rt_hw_stack_init(rt_uint8_t *stack_addr,void *tentry,void *parameter,void *texit)
// #if portUSING_MPU_WRAPPERS
// StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
// #else
// StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
// #endif
{
	StackType_t *sp, *tp;
	XtExcFrame  *frame;
	#if XCHAL_CP_NUM > 0
	uint32_t *p;
	#endif
	uint32_t *threadptr;
	void *task_thread_local_start;
	extern int _thread_local_start, _thread_local_end, _rodata_start;

	uint32_t thread_local_sz = (uint8_t *)&_thread_local_end - (uint8_t *)&_thread_local_start;

	thread_local_sz = ALIGNUP(0x10, thread_local_sz);

	/* Initialize task's stack so that we have the following structure at the top:

		----LOW ADDRESSES ----------------------------------------HIGH ADDRESSES----------
		 task stack | interrupt stack frame | thread local vars | co-processor save area |
		----------------------------------------------------------------------------------
					|																	 |
					SP 																stack_addr

		All parts are aligned to 16 byte boundary. */
	sp = (StackType_t *) (((UBaseType_t)(stack_addr + 1) - XT_CP_SIZE - thread_local_sz - XT_STK_FRMSZ) & ~0xf);

	/* Clear the entire frame (do not use memset() because we don't depend on C library) */
	for (tp = sp; tp <= stack_addr; ++tp)
		*tp = 0;

	frame = (XtExcFrame *) sp;

	/* Explicitly initialize certain saved registers */
	#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
	frame->pc	= (UBaseType_t) vPortTaskWrapper;	/* task wrapper						*/
	#else
	frame->pc   = (UBaseType_t) tentry;				/* task entrypoint					*/
	#endif
	frame->a0	= 0;								/* to terminate GDB backtrace		*/
	frame->a1	= (UBaseType_t) sp + XT_STK_FRMSZ;	/* physical top of stack frame		*/
	frame->exit = (UBaseType_t) _xt_user_exit;		/* user exception exit dispatcher	*/

	/* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
	/* Also set entry point argument parameter. */
	#ifdef __XTENSA_CALL0_ABI__
		#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
		frame->a2 = (UBaseType_t) tentry;
		frame->a3 = (UBaseType_t) parameter;
		#else
		frame->a2 = (UBaseType_t) parameter;
		#endif
	frame->ps = PS_UM | PS_EXCM;
	#else
	/* + for windowed ABI also set WOE and CALLINC (pretend task was 'call4'd). */
		#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
		frame->a6 = (UBaseType_t) tentry;
		frame->a7 = (UBaseType_t) parameter;
		#else
		frame->a6 = (UBaseType_t) parameter;
		#endif
	frame->ps = PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1);
	#endif

	#ifdef XT_USE_SWPRI
	/* Set the initial virtual priority mask value to all 1's. */
	frame->vpri = 0xFFFFFFFF;
	#endif

	/* Init threadptr reg and TLS vars */
	task_thread_local_start = (void *)(((uint32_t)stack_addr - XT_CP_SIZE - thread_local_sz) & ~0xf);
	memcpy(task_thread_local_start, &_thread_local_start, thread_local_sz);
	threadptr = (uint32_t *)(sp + XT_STK_EXTRA);
	/* shift threadptr by the offset of _thread_local_start from DROM start;
	   need to take into account extra 16 bytes offset */
	*threadptr = (uint32_t)task_thread_local_start - ((uint32_t)&_thread_local_start - (uint32_t)&_rodata_start) - 0x10;

	#if XCHAL_CP_NUM > 0
	/* Init the coprocessor save area (see xtensa_context.h) */
	/* No access to TCB here, so derive indirectly. Stack growth is top to bottom.
         * //p = (uint32_t *) xMPUSettings->coproc_area;
	 */
	p = (uint32_t *)(((uint32_t) stack_addr - XT_CP_SIZE) & ~0xf);
	p[0] = 0;
	p[1] = 0;
	p[2] = (((uint32_t) p) + 12 + XCHAL_TOTAL_SA_ALIGN - 1) & -XCHAL_TOTAL_SA_ALIGN;
	#endif

	return sp;
}



void rt_hw_console_output(const char *str)
{
    /* empty console output */
    ets_printf(str);
}





