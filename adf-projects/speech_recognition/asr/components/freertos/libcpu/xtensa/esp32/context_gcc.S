#define NOERROR #
NOERROR: .error "C preprocessor needed for this file: make sure its filename\
 ends in uppercase .S, or use xt-xcc's -x assembler-with-cpp option."

// #include "freertos/xtensa_rtos.h"
// #include "freertos/xtensa_context.h"
#include "xtensa_context.h"
#include "xtensa_rtos.h"
#include "sdkconfig.h"

#ifdef XT_USE_OVLY
#include <xtensa/overlay_os_asm.h>
#endif


// #include <rthw.h>
// #include <rtthread.h>


//在中断中切换的标志
.extern rt_uint32_t rt_thread_switch_interrupt_flag;
//from 和 to 线程
//临时变量 可能用不到
.extern rt_uint32_t rt_interrupt_from_thread;
.extern rt_uint32_t rt_interrupt_to_thread;




    .text

//线程中 上下文切换
/*
 * void rt_hw_context_switch(rt_ubase_t from, rt_ubase_t to);
 *
 * a2 --> from
 * a3 --> to
 */
    .global rt_hw_context_switch
    .type   rt_hw_context_switch,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch:
    ENTRY0

    l32i    a2,  a2, 0 // 取出from thread的ps指针
    l32i    a3,  a3, 0 // 取出to thread的ps指针


    //给全局变量 rt_interrupt_from_thread 赋值
    movi    a4,  rt_interrupt_from_thread
    s32i    a2,  a4, 0

    //给全局变量 rt_interrupt_to_thread 赋值
    movi    a4,  rt_interrupt_to_thread
    s32i    a3,  a4, 0

    //call4 rt_debug_test2

    //开始调度
    #ifdef __XTENSA_CALL0_ABI__
    call0   vPortYield
    #else
    call4   vPortYield
    #endif




//     //判断 rt_thread_switch_interrupt_flag 是否为 1
//     movi    a4,  rt_thread_switch_interrupt_flag       /* address of switch flag          */
//     l32i    a5,  a4, 0                  /* a5 = rt_thread_switch_interrupt_flag           */
//     bnez    a5,  .reswitch             /* flag != 0 则直接跳转   */
//     movi    a5,  1
//     s32i    a5,  a4, 0                  /* rt_thread_switch_interrupt_flag 赋值为 1 表示需要调度 */

//     //给全局变量 rt_interrupt_from_thread 赋值
//     movi    a4,  rt_interrupt_from_thread
//     s32i    a2,  a4, 0

// .reswitch:
//     //给全局变量 rt_interrupt_to_thread 赋值
//     movi    a4,  rt_interrupt_to_thread
//     s32i    a3,  a4, 0

//     //call4 rt_debug_test2

//     //开始调度
//     #ifdef __XTENSA_CALL0_ABI__
//     call0   vPortYield
//     #else
//     call4   vPortYield
//     #endif



    // #ifdef __XTENSA_CALL0_ABI__
    // call0   vPortYieldFromInt       /* call dispatch inside the function; never returns */
    // #else
    // call4   vPortYieldFromInt       /* this one returns */
    // call0   _frxt_dispatch          /* tail-call dispatcher */
    // #endif

    RET0

//中断中 上下文切换
/*
 * void rt_hw_context_switch_interrupt(rt_uint32 from, rt_uint32 to);
 * a2 --> from
 * a3 --> to
 */
    .global rt_hw_context_switch_interrupt
    .type   rt_hw_context_switch_interrupt,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch_interrupt:
    ENTRY0

    l32i    a2,  a2, 0 // 取出from thread的ps指针
    l32i    a3,  a3, 0 // 取出to thread的ps指针

    //判断 rt_thread_switch_interrupt_flag 是否为 1
    movi    a4,  rt_thread_switch_interrupt_flag       /* address of switch flag          */
    l32i    a5,  a4, 0                  /* a5 = rt_thread_switch_interrupt_flag           */
    bnez    a5,  .reswitch1             /* flag != 0 则直接跳转   */
    movi    a5,  1
    s32i    a5,  a4, 0                  /* rt_thread_switch_interrupt_flag 赋值为 1 表示需要调度 */

    //给全局变量 rt_interrupt_from_thread 赋值
    movi    a4,  rt_interrupt_from_thread
    s32i    a2,  a4, 0

.reswitch1:
    //给全局变量 rt_interrupt_to_thread 赋值
    movi    a4,  rt_interrupt_to_thread
    s32i    a3,  a4, 0

    //开始调度
    //这里不应该立马调度 只应该置位标志位 rt_thread_switch_interrupt_flag 即可 等待中断结束 会自动调度
    // #ifdef __XTENSA_CALL0_ABI__
    // call0   vPortYieldFromInt       /* call dispatch inside the function; never returns */
    // #else
    // call4   vPortYieldFromInt       /* this one returns */
    // call0   _frxt_dispatch          /* tail-call dispatcher */

    RET0


//切换到第一个线程
//注意 rtt 中传入参数均为 sp 指针
/*
 * void rt_hw_context_switch_to(rt_uint32 to);
 * a2 --> to
 */
    .global rt_hw_context_switch_to
    .type   rt_hw_context_switch_to,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch_to:
    ENTRY0                                  /* a2 = to thread */
/*
    #ifdef __XTENSA_CALL0_ABI__
    call0   rt_debug_printf_a2a3
    #else
    call4   rt_debug_printf_a2a3
    #endif
*/
    l32i    a2,  a2, 0 // 取出to thread的ps指针

/*
    #ifdef __XTENSA_CALL0_ABI__
    call0   rt_debug_printf_a2a3
    #else
    call4   rt_debug_printf_a2a3
    #endif
*/

    //保存到 rt_interrupt_to_thread
    movi    a3,  rt_interrupt_to_thread
    s32i    a2,  a3, 0

    //清空 rt_interrupt_from_thread
    movi    a3,  rt_interrupt_from_thread
    movi    a2,  0
    s32i    a2,  a3, 0

    // //设置中断标志位 rt_thread_switch_interrupt_flag 为 1
    // movi    a3,  rt_thread_switch_interrupt_flag
    // movi    a2,  1
    // s32i    a2,  a3, 0

    //开始调度
    #ifdef __XTENSA_CALL0_ABI__
    call0   vPortYieldFromInt       /* call dispatch inside the function; never returns */
    #else
    call4   vPortYieldFromInt       /* this one returns */
    call0   _frxt_dispatch          /* tail-call dispatcher */
    /* Never returns here. */
    #endif

    RET0

