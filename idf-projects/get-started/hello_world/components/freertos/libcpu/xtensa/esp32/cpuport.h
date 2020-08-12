#ifndef CPUPORT_H__
#define CPUPORT_H__

#include <rtconfig.h>




// #include <stdint.h>
// #include <stdlib.h>
// #include <stdbool.h>

#include <xtensa/hal.h>
#include <xtensa/config/core.h>
#include <xtensa/config/system.h>	/* required for XSHAL_CLIB */
#include <xtensa/xtruntime.h>
#include "esp_private/crosscore_int.h"
#include "esp_timer.h"              /* required for FreeRTOS run time stats */


#include <esp_heap_caps.h>

#include "sdkconfig.h"

#ifdef CONFIG_LEGACY_INCLUDE_COMMON_HEADERS
#include "soc/soc_memory_layout.h"
#endif



#include "esp_attr.h"











//使能中断
#define rt_hw_interrupt_enable(state)     do { XTOS_RESTORE_JUST_INTLEVEL(state); } while (0)
//关闭中断
static inline unsigned portENTER_CRITICAL_NESTED() {
	return XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL);
}
#define rt_hw_interrupt_disable()            portENTER_CRITICAL_NESTED()




void rt_hw_console_output(const char *str);

#endif
