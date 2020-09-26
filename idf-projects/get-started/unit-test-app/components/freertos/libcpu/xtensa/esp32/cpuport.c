#include <rthw.h>
#include <rtthread.h>

#include "cpuport.h"


#include <stdlib.h>
#include <string.h>
#include <xtensa/config/core.h>

#include "xtensa_rtos.h"

#include "esp32/rom/ets_sys.h"
#include "soc/cpu.h"
#include "esp32/rom/uart.h"
#include "driver/uart.h"
#include "driver/uart_select.h"

#include "freertos/portmacro.h"
#include "FreeRTOS.h"
#include "task.h"

#include "esp_debug_helpers.h"
#include "esp_private/crosscore_int.h"

#include "esp_intr_alloc.h"
#include "esp_log.h"


/* Align a value up to nearest n-byte boundary, where n is a power of 2. */
#define ALIGNUP(n, val) (((val) + (n)-1) & -(n))

typedef rt_uint8_t			StackType_t;
typedef unsigned int	    UBaseType_t;
typedef void (*TaskFunction_t)( void * );



volatile rt_ubase_t  rt_interrupt_from_thread = 0;
volatile rt_ubase_t  rt_interrupt_to_thread   = 0;
volatile rt_uint32_t rt_thread_switch_interrupt_flag = 0;



// User exception dispatcher when exiting
void _xt_user_exit(void);

#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
// Wrapper to allow task functions to return (increases stack overhead by 16 bytes)
static void vPortTaskWrapper(TaskFunction_t pxCode, void *pvParameters)
{
	pxCode(pvParameters);
	//RT-thread tasks should not return. Log the task name and abort.
	char * pcTaskName = pcTaskGetTaskName(NULL);
	ESP_LOGE("RT-thread", "RT-thread thread \"%s\" should not return, Aborting now!", pcTaskName);
	abort();
}
#endif



//宏 tskSTACK_FILL_BYTE == 0xa5U （tasks.c）会指定填充的内容，防止溢出

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
	// #if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
	// frame->pc	= (UBaseType_t) vPortTaskWrapper;	/* task wrapper						*/
	// #else
	frame->pc   = (UBaseType_t) tentry;				/* task entrypoint					*/
	// #endif
	frame->a0	= 0;								/* to terminate GDB backtrace		*/
	frame->a1	= (UBaseType_t) sp + XT_STK_FRMSZ;	/* physical top of stack frame		*/
	frame->exit = (UBaseType_t) _xt_user_exit;		/* user exception exit dispatcher	*/

    // rt_kprintf("frame addr:0x%08x exit[0x%08x] addr:0x%08x pc addr:0x%08x\n", frame, frame->exit, &frame->exit, &frame->pc);// by jz

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


// rt-thread 中的控制台输出
void rt_hw_console_output(const char *str)
{
    /* empty console output */
    ets_printf(str);
}
// rt-thread 中的finsh组件的输入
char rt_hw_console_getchar(void)
{
    rt_uint16_t len = 0;
    char ch = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t*)&len));
    if(len > 0)
    {
        uart_read_bytes(UART_NUM_0, (uint8_t *)&ch, 1, 100);
        return ch;
    }
    return -1;
}

//使能中断
void rt_hw_interrupt_enable(rt_base_t level)
{
    portEXIT_CRITICAL_NESTED(level);
    // XTOS_RESTORE_JUST_INTLEVEL(level);
}


//关闭中断
rt_base_t rt_hw_interrupt_disable(void)
{
    return portENTER_CRITICAL_NESTED();
}

struct rt_thread *rt_from_thread;

// 找到上一个线程
void rt_find_from_thread( void )
{
    // rt_thread_self();
    rt_from_thread = rt_list_entry(rt_interrupt_from_thread,
                                  struct rt_thread,
                                  sp);
    rt_kprintf("find from_thread:[%s][0x%08x] sp:[0x%08x] exit:[0x%08x]\n", rt_from_thread->name, rt_from_thread, rt_interrupt_from_thread, *(rt_uint32_t *)rt_interrupt_from_thread);
}

//调试函数
void rt_debug_printf_a2(rt_uint32_t a2)
{
    rt_kprintf("A2: 0x%08x\n", a2);
}
void rt_debug_printf_a2a3(rt_uint32_t a2, rt_uint32_t a3)
{
    rt_kprintf("A2: 0x%08x A3: 0x%08x\n", a2, a3);
}
void rt_debug_test1(void)
{
    rt_kprintf("test point 1\n");
}
void rt_debug_test2(void)
{
    rt_kprintf("test point 2\n");
}