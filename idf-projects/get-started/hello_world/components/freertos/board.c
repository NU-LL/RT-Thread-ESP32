#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>

//----------------------------------------------------------------
//这里应该是时钟初始化
//----------------------------------------------------------------


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
    /* System Clock Update */
    // SystemCoreClockUpdate();

    /* System Tick Configuration */
    // _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

// void SysTick_Handler(void)
// {
//     /* enter interrupt */
//     rt_interrupt_enter();

//     rt_tick_increase();

//     /* leave interrupt */
//     rt_interrupt_leave();
// }
