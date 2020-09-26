#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include "soc/cpu.h"
#include "soc/rtc.h"
#include "soc/rtc_wdt.h"
#include "soc/timer_periph.h"
#include "soc/rtc_periph.h"

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 1024
static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 4K(1024 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    extern void _xt_tick_divisor_init(void);
    extern void _frxt_tick_timer_init(void);

    //初始化硬件定时器（systick）
    /* Init the tick divisor value */
	_xt_tick_divisor_init();
	/* Setup the hardware to generate the tick. */
	_frxt_tick_timer_init();

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}





#define THREAD_PRIORITY         15
#define THREAD_STACK_SIZE       2048
#define THREAD_TIMESLICE        10



extern rt_uint32_t rt_interrupt_from_thread;
extern rt_uint32_t rt_interrupt_to_thread;

static void thread1_entry(void *parameter)
{
    rt_uint32_t count = 0;

    while (1)
    {
        /* 线程 1 采用低优先级运行，一直打印计数值 */
        rt_kprintf("thread1 count: %d\n", count ++);
        // rt_kprintf("\n\nthread1 count: %d\n\n", count ++);
        rt_thread_mdelay(1000);
        // rt_kprintf("from thread sp:[0x%08x] exit:[0x%08x]\n", rt_interrupt_from_thread, *(rt_uint32_t *)rt_interrupt_from_thread);
    }
}

int main(void)
{
    // rt_uint32_t temp = 0x12345678;

    // rt_thread_t tid1 = RT_NULL;
    // tid1 = rt_thread_create("thread1",
    //                         thread1_entry, RT_NULL,
    //                         THREAD_STACK_SIZE,
    //                         THREAD_PRIORITY, THREAD_TIMESLICE);
    // if (tid1 != RT_NULL)
    //     rt_thread_startup(tid1);

    while(1)
    {
        // rt_kprintf(".");
        // rt_kprintf("Hello World!\n");
        // rt_kprintf("\n\n[main]:Hello World!\n\n");
        // rt_kprintf("from thread sp:[0x%08x] exit:[0x%08x]\n", rt_interrupt_from_thread, *(rt_uint32_t *)rt_interrupt_from_thread);
        // rt_kprintf("0x%08x 0x%08x 0x%08x\n", *((rt_uint32_t *)(rt_thread_self()->sp)-1), *((rt_uint32_t *)(rt_thread_self()->sp)), *((rt_uint32_t *)(rt_thread_self()->sp)+1));
        // rt_kprintf("\n\n\nsp:0x%08x addr:0x%08x\n", (rt_thread_self()->stack_addr), &temp);
        // rt_thread_delay(100);
        rt_thread_mdelay(1000);
    }
    return RT_EOK;
}

