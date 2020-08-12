

#define NOERROR #
NOERROR: .error "C preprocessor needed for this file: make sure its filename\
 ends in uppercase .S, or use xt-xcc's -x assembler-with-cpp option."

#include "freertos/xtensa_rtos.h"
#include "freertos/xtensa_context.h"

#ifdef XT_USE_OVLY
#include <xtensa/overlay_os_asm.h>
#endif

    .text






/*
 * void rt_hw_context_switch(rt_ubase_t from, rt_ubase_t to);
 *
 * a0 --> from
 * a1 --> to
 */
    .global rt_hw_context_switch
    .type   rt_hw_context_switch,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch:


    ret


/*
 * void rt_hw_context_switch_interrupt(rt_uint32 from, rt_uint32 to);
 * R0 --> from
 * R1 --> to
 */
    .global rt_hw_context_switch_interrupt
    .type   rt_hw_context_switch_interrupt,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch_interrupt:


    ret



/*
 * void rt_hw_context_switch_to(rt_uint32 to);
 * R0 --> to
 */
    .global rt_hw_context_switch_to
    .type   rt_hw_context_switch_to,@function
    .align  4
	.literal_position
    .align  4
rt_hw_context_switch_to:


    ret

