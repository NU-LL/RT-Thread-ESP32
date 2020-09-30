/*
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
*/

#include "xtensa_rtos.h"
#include "sdkconfig.h"

#define TOPOFSTACK_OFFS                 0x00    /* StackType_t *pxTopOfStack */
#define CP_TOPOFSTACK_OFFS              0x04    /* xMPU_SETTINGS.coproc_area */
//rtt 中线程栈栈顶指针在tcb中的偏移
#ifdef RT_USING_MODULE
//默认 ESP32 编译器默认的对齐数为 4
//这里需要提前测试一下
#define RTT_TOPOFSTACK_OFFS             0x18    /* void       *sp; 默认4字节对齐：4+4+8+8=0x18  8字节对齐：4+4+8+8=0x18  */
#else
//#define RTT_TOPOFSTACK_OFFS             0x14    /* void       *sp; 默认4字节对齐：4+8+8  默认8字节对齐：8+8+8=0x18  */
#define RTT_TOPOFSTACK_OFFS             0x1C
#endif
//协处理器暂时不考虑
#define RTT_CP_TOPOFSTACK_OFFS          0x04    /* xMPU_SETTINGS.coproc_area */

.extern pxCurrentTCB
//当前线程的控制块
.extern struct rt_thread *rt_current_thread
//上一个线程的控制块
.extern struct rt_thread *rt_from_thread
.extern struct rt_thread *from_thread

//注意传过来的是地址
//中断嵌套层数
.extern rt_uint8_t rt_interrupt_nest;
//在中断中切换的标志
.extern rt_uint32_t rt_thread_switch_interrupt_flag;
//from 和 to 线程的 sp 指针
//注意：这里均为 sp 指针
.extern rt_uint32_t rt_interrupt_from_thread;
.extern rt_uint32_t rt_interrupt_to_thread;
/*
*******************************************************************************
* Interrupt stack. The size of the interrupt stack is determined by the config
* parameter "configISR_STACK_SIZE" in FreeRTOSConfig.h
*******************************************************************************
*/

    .data
    .align      16
    .global     port_IntStack
    .global     port_IntStackTop
    .global     port_switch_flag
port_IntStack:
    .space      configISR_STACK_SIZE*portNUM_PROCESSORS		/* This allocates stacks for each individual CPU. */
port_IntStackTop:
    .word		0
port_switch_flag:
    .space      portNUM_PROCESSORS*4 /* One flag for each individual CPU. */

    .text

/*
*******************************************************************************
* _frxt_setup_switch
* void _frxt_setup_switch(void);
*
* Sets an internal flag indicating that a task switch is required on return
* from interrupt handling.
*
*******************************************************************************
*/
//置位中断标志位 rt_thread_switch_interrupt_flag
//宏 portYIELD_FROM_ISR 会调用该函数
// 该宏需要修改！！！
//或许可以将该宏直接改为调度函数 由rtt内核判断在中断中 然后进行调度
//这里先放着
    .global     _frxt_setup_switch
    .type       _frxt_setup_switch,@function
    .align      4
_frxt_setup_switch:

    ENTRY(16)

	getcoreid a3
    movi    a2, port_switch_flag
	addx4	a2,  a3, a2

    movi    a3, 1
    s32i    a3, a2, 0

    RET(16)




/*
*******************************************************************************
* _frxt_int_enter
* void _frxt_int_enter(void)
*
* Implements the Xtensa RTOS porting layer's XT_RTOS_INT_ENTER function for
* freeRTOS. Saves the rest of the interrupt context (not already saved).
* May only be called from assembly code by the 'call0' instruction, with
* interrupts disabled.
* See the detailed description of the XT_RTOS_ENTER macro in xtensa_rtos.h.
*
*******************************************************************************
*/
//进入中断
//本应该调用 rt_interrupt_enter
//但是这里只能汇编 C的环境还没有配置好
    .globl  _frxt_int_enter
    .type   _frxt_int_enter,@function
    .align  4
_frxt_int_enter:

    /* Save a12-13 in the stack frame as required by _xt_context_save. */
    s32i    a12, a1, XT_STK_A12
    s32i    a13, a1, XT_STK_A13

    /* Save return address in a safe place (free a0). */
    mov     a12, a0

    /* Save the rest of the interrupted context (preserves A12-13). */
    call0   _xt_context_save

    // //rt_interrupt_nest ++; 8bit 变量
    movi    a3,  rt_interrupt_nest
    l32i    a2,  a3, 0                  /* a2 = rt_interrupt_nest          */
    addi    a2,  a2, 1                  /* increment nesting count         */
    s32i    a2,  a3, 0                  /* save nesting count              */

    //由于xtensa架构没有 msp和 psp
    //这里来设置中断栈 之后中断中才能调用C函数
    movi    a2,  rt_current_thread
    l32i    a2,  a2, 0                  /* a2 = current TCB                */
    beqz    a2,  1f
    s32i    a1,  a2, RTT_TOPOFSTACK_OFFS    /* pxCurrentTCB->sp = SP */
    movi    a1,  port_IntStack+configISR_STACK_SIZE   /* a1 = top of intr stack for CPU 0  */
    //a1 = port_IntStack+configISR_STACK_SIZE出地址 即 a1(sp) 此时为中断栈的栈顶
    //此时中断的C语言环境设置完毕 之后可以进入C函数

.Lnested:
1:
    mov     a0,  a12                    /* restore return addr and return  */
    ret

/*
*******************************************************************************
* _frxt_int_exit
* void _frxt_int_exit(void)
*
* Implements the Xtensa RTOS porting layer's XT_RTOS_INT_EXIT function for
* FreeRTOS. If required, calls vPortYieldFromInt() to perform task context
* switching, restore the (possibly) new task's context, and return to the
* exit dispatcher saved in the task's stack frame at XT_STK_EXIT.
* May only be called from assembly code by the 'call0' instruction. Does not
* return to caller.
* See the description of the XT_RTOS_ENTER macro in xtensa_rtos.h.
*
*******************************************************************************
*/
//退出中断
//本应该调用 rt_interrupt_leave
//同时应该开始调度（没有pendSV）
    .globl  _frxt_int_exit
    .type   _frxt_int_exit,@function
    .align  4
_frxt_int_exit:

    //rt_interrupt_nest --;
    movi    a3,  rt_interrupt_nest
    rsil    a0,  XCHAL_EXCM_LEVEL       /* lock out interrupts             */
    l32i    a2,  a3, 0                  /* a2 = rt_interrupt_nest          */
    addi    a2,  a2, -1                 /* decrement nesting count         */
    s32i    a2,  a3, 0                  /* save nesting count              */
    bnez    a2,  .Lnesting              /* !=0 after decr so still nested  */ //如果嵌套 直接退出

    //置位标志位 方便 vPortYieldFromInt (rt_hw_context_switch_interrupt) 中调度
    movi    a2,  rt_thread_switch_interrupt_flag       /* address of switch flag          */
    l32i    a3,  a2, 0                  /* a3 = rt_thread_switch_interrupt_flag           */
    beqz    a3,  .Lnoswitch             /* flag = 0 means no switch reqd   */       // 根据 rt_thread_switch_interrupt_flag 的值决定中断后是否需要切换上下文

    movi    a3,  0
    s32i    a3,  a2, 0                  /* zero out the flag for next time */

    //这里表示需要切换上下文
1:
    /*
    Call0 ABI callee-saved regs a12-15 need to be saved before possible preemption.
    However a12-13 were already saved by _frxt_int_enter().
    */
    #ifdef __XTENSA_CALL0_ABI__
    s32i    a14, a1, XT_STK_A14
    s32i    a15, a1, XT_STK_A15
    #endif

    #ifdef __XTENSA_CALL0_ABI__
    call0   vPortYieldFromInt       /* call dispatch inside the function; never returns */
    #else
    call4   vPortYieldFromInt       /* this one returns */
    call0   _frxt_dispatch          /* tail-call dispatcher */
    /* Never returns here. */
    #endif

    //退出中断 这里表示不需要切换上下文
.Lnoswitch:
    /*
    If we came here then about to resume the interrupted task.
    */

.Lnesting:
    /*
    We come here only if there was no context switch, that is if this
    is a nested interrupt, or the interrupted task was not preempted.
    In either case there's no need to load the SP.
    */

    //如果当前线程为空 则进行调度
    //rtt 中是在 Pend SV中根据 rt_interrupt_from_thread 是否为 0 来判断是否为第一个线程
    //这里可能是判断第一个线程的 还需要仔细考虑
    movi    a2,  rt_current_thread
    l32i    a2,  a2, 0                  /* a2 = current TCB                */
    beqz    a2,  1b                     /* no task ? go to dispatcher      */  //如果 rt_current_thread=0 则进行调度（考虑到这种情况或许是freertos中调用第一个线程？）
    l32i    a1,  a2, RTT_TOPOFSTACK_OFFS    /* SP = pxCurrentTCB->sp */

    /* Restore full context from interrupt stack frame */
    call0   _xt_context_restore

    /*
    Must return via the exit dispatcher corresponding to the entrypoint from which
    this was called. Interruptee's A0, A1, PS, PC are restored and the interrupt
    stack frame is deallocated in the exit dispatcher.
    */
    l32i    a0,  a1, XT_STK_EXIT
    ret

/*
**********************************************************************************************************
*                                           _frxt_timer_int
*                                      void _frxt_timer_int(void)
*
* Implements the Xtensa RTOS porting layer's XT_RTOS_TIMER_INT function for FreeRTOS.
* Called every timer interrupt.
* Manages the tick timer and calls xPortSysTickHandler() every tick.
* See the detailed description of the XT_RTOS_ENTER macro in xtensa_rtos.h.
*
* Callable from C (obeys ABI conventions). Implemented in assmebly code for performance.
*
**********************************************************************************************************
*/
    .globl  _frxt_timer_int
    .type   _frxt_timer_int,@function
    .align  4
_frxt_timer_int:

    /*
    Xtensa timers work by comparing a cycle counter with a preset value.  Once the match occurs
    an interrupt is generated, and the handler has to set a new cycle count into the comparator.
    To avoid clock drift due to interrupt latency, the new cycle count is computed from the old,
    not the time the interrupt was serviced. However if a timer interrupt is ever serviced more
    than one tick late, it is necessary to process multiple ticks until the new cycle count is
    in the future, otherwise the next timer interrupt would not occur until after the cycle
    counter had wrapped (2^32 cycles later).

    do {
        ticks++;
        old_ccompare = read_ccompare_i();
        write_ccompare_i( old_ccompare + divisor );
        service one tick;
        diff = read_ccount() - old_ccompare;
    } while ( diff > divisor );
    */

    ENTRY(16)

    #ifdef CONFIG_PM_TRACE
    movi a6, 1 /* = ESP_PM_TRACE_TICK */
    getcoreid a7
    call4 esp_pm_trace_enter
    #endif // CONFIG_PM_TRACE

.L_xt_timer_int_catchup:

    /* Update the timer comparator for the next tick. */
    #ifdef XT_CLOCK_FREQ
    movi    a2, XT_TICK_DIVISOR         /* a2 = comparator increment          */
    #else
    movi    a3, _xt_tick_divisor
    l32i    a2, a3, 0                   /* a2 = comparator increment          */
    #endif
    rsr     a3, XT_CCOMPARE             /* a3 = old comparator value          */
    add     a4, a3, a2                  /* a4 = new comparator value          */
    wsr     a4, XT_CCOMPARE             /* update comp. and clear interrupt   */
    esync

    #ifdef __XTENSA_CALL0_ABI__
    /* Preserve a2 and a3 across C calls. */
    s32i    a2, sp, 4
    s32i    a3, sp, 8
    #endif

    /* Call the FreeRTOS tick handler (see port.c). */
    #ifdef __XTENSA_CALL0_ABI__
    call0   xPortSysTickHandler
    #else
    call4   xPortSysTickHandler
    #endif

    #ifdef __XTENSA_CALL0_ABI__
    /* Restore a2 and a3. */
    l32i    a2, sp, 4
    l32i    a3, sp, 8
    #endif

    /* Check if we need to process more ticks to catch up. */
    esync                               /* ensure comparator update complete  */
    rsr     a4, CCOUNT                  /* a4 = cycle count                   */
    sub     a4, a4, a3                  /* diff = ccount - old comparator     */
    blt     a2, a4, .L_xt_timer_int_catchup  /* repeat while diff > divisor */

#ifdef CONFIG_PM_TRACE
    movi a6, 1 /* = ESP_PM_TRACE_TICK */
    getcoreid a7
    call4 esp_pm_trace_exit
#endif // CONFIG_PM_TRACE

    RET(16)

    /*
**********************************************************************************************************
*                                           _frxt_tick_timer_init
*                                      void _frxt_tick_timer_init(void)
*
* Initialize timer and timer interrrupt handler (_xt_tick_divisor_init() has already been been called).
* Callable from C (obeys ABI conventions on entry).
*
**********************************************************************************************************
*/
    .globl  _frxt_tick_timer_init
    .type   _frxt_tick_timer_init,@function
    .align  4
_frxt_tick_timer_init:

    ENTRY(16)


    /* Set up the periodic tick timer (assume enough time to complete init). */
    #ifdef XT_CLOCK_FREQ
    movi    a3, XT_TICK_DIVISOR
    #else
    movi    a2, _xt_tick_divisor
    l32i    a3, a2, 0
    #endif
    rsr     a2, CCOUNT              /* current cycle count */
    add     a2, a2, a3              /* time of first timer interrupt */
    wsr     a2, XT_CCOMPARE         /* set the comparator */

    /*
    Enable the timer interrupt at the device level. Don't write directly
    to the INTENABLE register because it may be virtualized.
    */
    #ifdef __XTENSA_CALL0_ABI__
    movi    a2, XT_TIMER_INTEN
    call0   xt_ints_on
    #else
    movi    a6, XT_TIMER_INTEN
    call4   xt_ints_on
    #endif

    RET(16)

/*
**********************************************************************************************************
*                                    DISPATCH THE HIGH READY TASK
*                                     void _frxt_dispatch(void)
*
* Switch context to the highest priority ready task, restore its state and dispatch control to it.
*
* This is a common dispatcher that acts as a shared exit path for all the context switch functions
* including vPortYield() and vPortYieldFromInt(), all of which tail-call this dispatcher
* (for windowed ABI vPortYieldFromInt() calls it indirectly via _frxt_int_exit() ).
*
* The Xtensa port uses different stack frames for solicited and unsolicited task suspension (see
* comments on stack frames in xtensa_context.h). This function restores the state accordingly.
* If restoring a task that solicited entry, restores the minimal state and leaves CPENABLE clear.
* If restoring a task that was preempted, restores all state including the task's CPENABLE.
*
* Entry:
*   pxCurrentTCB  points to the TCB of the task to suspend,
*   Because it is tail-called without a true function entrypoint, it needs no 'entry' instruction.
*
* Exit:
*   If incoming task called vPortYield() (solicited), this function returns as if from vPortYield().
*   If incoming task was preempted by an interrupt, this function jumps to exit dispatcher.
*
**********************************************************************************************************
*/
//真正的调度函数
//类似于 Cortex M 中的 PendSV
    .globl  _frxt_dispatch
    .type   _frxt_dispatch,@function
    .align  4
_frxt_dispatch:

    //这里和 Cortex M 中的 PendSV 不太一样
    //Cortex M 中中断开始后硬件会自动保存一部分必要的上文，然后中断结束后恢复这部分上文
    //涉及到任务调度时，中断结束后会到Pend SV中断中，在这里边会根据 switch_to_thread 和 rt_interrupt_from_thread 保存剩下硬件未自动保存的上文 紧接着就会蟹黄下文、恢复下文 然后切换到新的任务并运行
    //
    //而xtensa中 中断开始后会将上文统一保存 中断退出再加载会原来的上文恢复到原来运行的地方（这里的目的是为了切换栈，由线程栈切换到 port_IntStack 代表的中断栈，这里中断有点类似一个更加高级点的线程）
    //涉及到任务调度时，中断结束后会进入该函数切换下文，然后恢复下文 此时就会切换到新的任务栈，然后运行新的任务（因为在中断开始时已经保存了上文，这里就无需再次保存上文了）
    //任务切换涉及两种：一种是在线程上下文切换 一种是中断上下文切换
    //线程上下文切换：通过调用 vPortYield --> 保存上文，关闭中断，清除XT_STK_EXIT --> _frxt_dispatch
    //中断上下文切换：直接置位标志量 rt_thread_switch_interrupt_flag 即可，然后等待下一个 systick 中断到来进行任务调度
    // 切换 sp 指针
    movi    a2,  rt_interrupt_to_thread // 就绪列表中的最高优先级任务的 sp 指针 接下来要调度到这里
    l32i    sp,  a2, 0

    /* Determine the type of stack frame. */
    l32i    a2,  sp, XT_STK_EXIT        /* exit dispatcher or solicited flag */
    bnez    a2,  .L_frxt_dispatch_stk
// 跳转到这里是 XT_STK_EXIT 中没有退出函数
// 当用户调用 vPortYield 时 会将 XT_STK_EXIT 清零
// SOL帧 表明这是任务间调度
.L_frxt_dispatch_sol:
    /* Solicited stack frame. Restore minimal context and return from vPortYield(). */
    l32i    a3,  sp, XT_SOL_PS
    #ifdef __XTENSA_CALL0_ABI__
    l32i    a12, sp, XT_SOL_A12
    l32i    a13, sp, XT_SOL_A13
    l32i    a14, sp, XT_SOL_A14
    l32i    a15, sp, XT_SOL_A15
    #endif
    l32i    a0,  sp, XT_SOL_PC
    //暂时不考虑协处理器
    // #if XCHAL_CP_NUM > 0
    // /* Ensure wsr.CPENABLE is complete (should be, it was cleared on entry). */
    // rsync
    // #endif
    /* As soons as PS is restored, interrupts can happen. No need to sync PS. */
    wsr     a3,  PS
    #ifdef __XTENSA_CALL0_ABI__
    addi    sp,  sp, XT_SOL_FRMSZ // XT_SOL_FRMSZ == 8
    ret
    #else
    retw
    #endif

// 跳转到这里是 XT_STK_EXIT 中存在退出函数
// 等恢复下文后直接从 XT_STK_EXIT 退出（一般为 _xt_user_exit 用于还原PC、PS、A0、A1(sp)）
// STK帧 表明这是由中断引起的调度
.L_frxt_dispatch_stk:
    //暂时不考虑协处理器
    // #if XCHAL_CP_NUM > 0
    // /* Restore CPENABLE from task's co-processor save area. */
    // movi    a3, pxCurrentTCB            /* cp_state =                       */
	// getcoreid a2
	// addx4	a3,  a2, a3
    // l32i    a3, a3, 0
    // l32i    a2, a3, CP_TOPOFSTACK_OFFS     /* StackType_t                       *pxStack; */
    // l16ui   a3, a2, XT_CPENABLE         /* CPENABLE = cp_state->cpenable;   */
    // wsr     a3, CPENABLE
    // #endif

    /* Interrupt stack frame. Restore full context and return to exit dispatcher. */
    call0   _xt_context_restore

    /* In Call0 ABI, restore callee-saved regs (A12, A13 already restored). */
    #ifdef __XTENSA_CALL0_ABI__
    l32i    a14, sp, XT_STK_A14
    l32i    a15, sp, XT_STK_A15
    #endif

    //暂时不考虑协处理器
    // #if XCHAL_CP_NUM > 0
    // /* Ensure wsr.CPENABLE has completed. */
    // rsync
    // #endif

    /*
    Must return via the exit dispatcher corresponding to the entrypoint from which
    this was called. Interruptee's A0, A1, PS, PC are restored and the interrupt
    stack frame is deallocated in the exit dispatcher.
    */
    l32i    a0, sp, XT_STK_EXIT
    ret


/*
**********************************************************************************************************
*                            PERFORM A SOLICTED CONTEXT SWITCH (from a task)
*                                        void vPortYield(void)
*
* This function saves the minimal state needed for a solicited task suspension, clears CPENABLE,
* then tail-calls the dispatcher _frxt_dispatch() to perform the actual context switch
*
* At Entry:
*   pxCurrentTCB  points to the TCB of the task to suspend
*   Callable from C (obeys ABI conventions on entry).
*
* Does not return to caller.
*
**********************************************************************************************************
*/
    .globl  vPortYield
    .type   vPortYield,@function
    .align  4
vPortYield:
    // 任务间调度只需用 SOL 帧保存即可
    #ifdef __XTENSA_CALL0_ABI__
    addi    sp,  sp, -XT_SOL_FRMSZ
    #else
    entry   sp,  XT_SOL_FRMSZ
    #endif
    rsr     a2,  PS
    s32i    a0,  sp, XT_SOL_PC
    s32i    a2,  sp, XT_SOL_PS

    #ifdef __XTENSA_CALL0_ABI__
    s32i    a12, sp, XT_SOL_A12         /* save callee-saved registers      */
    s32i    a13, sp, XT_SOL_A13
    s32i    a14, sp, XT_SOL_A14
    s32i    a15, sp, XT_SOL_A15
    #else
    /* Spill register windows. Calling xthal_window_spill() causes extra    */
    /* spills and reloads, so we will set things up to call the _nw version */
    /* instead to save cycles.                                              */
    movi    a6,  ~(PS_WOE_MASK|PS_INTLEVEL_MASK)  /* spills a4-a7 if needed */
    and     a2,  a2, a6                           /* clear WOE, INTLEVEL    */
    addi    a2,  a2, XCHAL_EXCM_LEVEL             /* set INTLEVEL           */
    wsr     a2,  PS
    rsync
    call0   xthal_window_spill_nw
    l32i    a2,  sp, XT_SOL_PS                    /* restore PS             */
    wsr     a2,  PS
    #endif
    rsil    a2,  XCHAL_EXCM_LEVEL       /* disable low/med interrupts       */
    // 暂不考虑协处理器
    // #if XCHAL_CP_NUM > 0
    // /* Save coprocessor callee-saved state (if any). At this point CPENABLE */
    // /* should still reflect which CPs were in use (enabled).                */
    // call0   _xt_coproc_savecs
    // #endif

    // movi    a2,  pxCurrentTCB
	// getcoreid a3
	// addx4	a2,  a3, a2
    // l32i    a2,  a2, 0                  /* a2 = pxCurrentTCB                */
    // movi    a3,  0
    // s32i    a3,  sp, XT_SOL_EXIT        /* 0 to flag as solicited frame     */
    // s32i    sp,  a2, TOPOFSTACK_OFFS    /* pxCurrentTCB->pxTopOfStack = SP  */

    // 将 XT_SOL_EXIT 置0
    // _frxt_dispatch 中根据该位置来区分这是 SOL 帧
    // 这里应该是过的 from 线程的 TCB 并修改其中的 sp 指针
    movi    a2,  from_thread
    l32i    a2,  a2, 0                  /* a2 = current TCB                */
    movi    a3,  0
    s32i    a3,  sp, XT_SOL_EXIT        /* 0 to flag as solicited frame     */  //把 0写到 栈中 XT_SOL_EXIT 位置 使返回函数为空 这里是为了之后调度进 .L_frxt_dispatch_sol 分支进行准备
    s32i    sp,  a2, RTT_TOPOFSTACK_OFFS    /* pxCurrentTCB->pxTopOfStack = SP  */

    // 暂不考虑协处理器
//     #if XCHAL_CP_NUM > 0
//     /* Clear CPENABLE, also in task's co-processor state save area. */
//     l32i    a2,  a2, CP_TOPOFSTACK_OFFS /* a2 = pxCurrentTCB->cp_state      */
//     movi    a3,  0
//     wsr     a3,  CPENABLE
//     beqz    a2,  1f
//     s16i    a3,  a2, XT_CPENABLE        /* clear saved cpenable             */
// 1:
//     #endif
    /* Tail-call dispatcher. */
    call0   _frxt_dispatch
    /* Never reaches here. */


/*
**********************************************************************************************************
*                         PERFORM AN UNSOLICITED CONTEXT SWITCH (from an interrupt)
*                                        void vPortYieldFromInt(void)
*
* This calls the context switch hook (removed), saves and clears CPENABLE, then tail-calls the dispatcher
* _frxt_dispatch() to perform the actual context switch.
*
* At Entry:
*   Interrupted task context has been saved in an interrupt stack frame at pxCurrentTCB->pxTopOfStack.
*   pxCurrentTCB  points to the TCB of the task to suspend,
*   Callable from C (obeys ABI conventions on entry).
*
* At Exit:
*   Windowed ABI defers the actual context switch until the stack is unwound to interrupt entry.
*   Call0 ABI tail-calls the dispatcher directly (no need to unwind) so does not return to caller.
*
**********************************************************************************************************
*/
//中断中调度
    .globl  vPortYieldFromInt
    .type   vPortYieldFromInt,@function
    .align  4
vPortYieldFromInt:

    ENTRY(16)

    //暂时不考虑协处理器
    // #if XCHAL_CP_NUM > 0
    // /* Save CPENABLE in task's co-processor save area, and clear CPENABLE.  */
    // movi    a3, pxCurrentTCB            /* cp_state =                       */
	// getcoreid a2
	// addx4	a3,  a2, a3
    // l32i    a3, a3, 0

    // l32i    a2, a3, CP_TOPOFSTACK_OFFS

    // rsr     a3, CPENABLE
    // s16i    a3, a2, XT_CPENABLE         /* cp_state->cpenable = CPENABLE;   */
    // movi    a3, 0
    // wsr     a3, CPENABLE                /* disable all co-processors        */
    // #endif

    #ifdef __XTENSA_CALL0_ABI__
    /* Tail-call dispatcher. */
    call0   _frxt_dispatch
    /* Never reaches here. */
    #else
    RET(16)
    #endif

/*
**********************************************************************************************************
*                                        _frxt_task_coproc_state
*                                   void _frxt_task_coproc_state(void)
*
* Implements the Xtensa RTOS porting layer's XT_RTOS_CP_STATE function for FreeRTOS.
*
* May only be called when a task is running, not within an interrupt handler (returns 0 in that case).
* May only be called from assembly code by the 'call0' instruction. Does NOT obey ABI conventions.
* Returns in A15 a pointer to the base of the co-processor state save area for the current task.
* See the detailed description of the XT_RTOS_ENTER macro in xtensa_rtos.h.
*
**********************************************************************************************************
*/
#if XCHAL_CP_NUM > 0
// a15 返回一个指向当前任务的协处理器状态保存区的指针
// 协处理器暂不考虑 这里直接返回 0
    .globl  _frxt_task_coproc_state
    .type   _frxt_task_coproc_state,@function
    .align  4
_frxt_task_coproc_state:
1:  movi    a15, 0
2:  ret

#endif /* XCHAL_CP_NUM > 0 */
