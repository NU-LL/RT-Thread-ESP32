#include <rthw.h>
#include <rtthread.h>

#include <stdlib.h>
#include <string.h>

#include "esp32/rom/ets_sys.h"
#include "esp_newlib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/projdefs.h"
#include "freertos/portmacro.h"
// #include "freertos/portmacro_priv.h"
#include "freertos/semphr.h"
#include "freertos/StackMacros.h"


/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE	( 0x23U ) // rt-thread中会填入0x23

// 为了兼容 newlib ，newlib中会通过该变量来判断系统是否已经启动
// 需要手动置为 pdTRUE
volatile BaseType_t xSchedulerRunning 		= pdFALSE;


/*-----------------------------------------------------------
 * TASK CREATION API
 *----------------------------------------------------------*/

/**
 * 创建具有指定相似性的新任务
 *
 * 此函数类似于xTaskCreate，但是允许在SMP系统中设置任务关联
 *
 * @param pvTaskCode 指向任务入口函数的指针。 该任务永不返回（即连续循环）
 *
 * @param pcName 任务名字
 *
 * @param usStackDepth 任务栈大小（bytes），与原本的freertos不同
 *
 * @param pvParameters 任务参数
 *
 * @param uxPriority 任务优先级。包含MPU支持的系统可以选择通过设置优先级参数的portPRIVILEGE_BIT位在
 * 特权（系统）模式下创建任务。 例如，要创建优先级为2的特权任务，应将uxPriority参数设置为（2 | portPRIVILEGE_BIT）
 *
 * @param pvCreatedTask 返回创建的任务句柄
 *
 * @param xCoreID 如果值为tskNO_AFFINITY，则创建的任务不会固定到任何CPU，并且调度程序可以在任何可用内核上运行它。
 * 其他值指示任务应固定到的CPU的索引号。 指定大于（portNUM_PROCESSORS-1）的值将导致函数失败
 *
 * @return 如果成功创建任务并将其添加到就绪列表，则为pdPASS，否则，在文件projdefs.h中定义的错误代码
 *
 * \ingroup Tasks
 */
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	BaseType_t xTaskCreatePinnedToCore(	TaskFunction_t pvTaskCode,
										const char * const pcName,
										const uint32_t usStackDepth,
										void * const pvParameters,
										UBaseType_t uxPriority,
										TaskHandle_t * const pvCreatedTask,//二级指针
										const BaseType_t xCoreID)
    {
        rt_thread_t tid = RT_NULL;
        UBaseType_t rtt_priority = RT_THREAD_PRIORITY_MAX - 1 - uxPriority;//configMAX_PRIORITIES;//rtt最低优先级为 RT_THREAD_PRIORITY_MAX-1   freertos最低优先级为0
        tid = rt_thread_create(pcName, pvTaskCode, pvParameters, usStackDepth, (rt_uint8_t)rtt_priority, 50);
        if (tid != RT_NULL)
        {
            if( ( void * ) pvCreatedTask != NULL )
            {
                *pvCreatedTask = ( TaskHandle_t ) tid;//直接传回线程tid 可能有bug！！！
            }
            rt_thread_startup(tid);
            return pdPASS;
        }
        return pdFAIL;
    }

#endif



/**
 * 创建具有指定相似性的新任务
 *
 * 此函数类似于xTaskCreateStatic，但允许在SMP系统中指定任务关联。
 *
 * @param pvTaskCode 指向任务入口函数的指针。 该任务永不返回（即连续循环）
 *
 * @param pcName 任务名字
 *
 * @param ulStackDepth 任务栈大小（bytes），与原本的freertos不同
 *
 * @param pvParameters 任务参数
 *
 * @param uxPriority 任务运行优先级
 *
 * @param pxStackBuffer 必须指向ulStackDepth深度的StackType_t类型数组，用于该任务的任务栈，从而无需动态分配堆栈
 *
 * @param pxTaskBuffer 必须指向StaticTask_t类型变量，用于保存任务数据结构，从而无需动态分配堆栈
 *
 * @param xCoreID 如果值为tskNO_AFFINITY，则创建的任务不会固定到任何CPU，并且调度程序可以在任何可用内核上运行它。
 * 其他值指示任务应固定到的CPU的索引号。 指定大于（portNUM_PROCESSORS-1）的值将导致函数失败
 *
 * @return 如果pxStackBuffer或pxTaskBuffer都不为NULL，则将创建任务并返回pdPASS。
 * 如果pxStackBuffer或pxTaskBuffer为NULL，则不会创建任务，并返回errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY。
 *
 * \ingroup Tasks
 */
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
	TaskHandle_t xTaskCreateStaticPinnedToCore(	TaskFunction_t pvTaskCode,
												const char * const pcName,
												const uint32_t ulStackDepth,
												void * const pvParameters,
												UBaseType_t uxPriority,
												StackType_t * const pxStackBuffer,
												StaticTask_t * const pxTaskBuffer,
												const BaseType_t xCoreID )
    {
        struct rt_thread* thread;

        if( ( pxTaskBuffer != NULL ) && ( pxStackBuffer != NULL ) )
		{
            thread = (struct rt_thread* )pxTaskBuffer;//直接强制类型转换，可能有bug
            UBaseType_t rtt_priority = RT_THREAD_PRIORITY_MAX - 1 - uxPriority;//configMAX_PRIORITIES;//rtt最低优先级为 RT_THREAD_PRIORITY_MAX-1   freertos最低优先级为0
            rt_thread_init(thread, pcName, pvTaskCode, pvParameters, (void *)pxStackBuffer, ulStackDepth, rtt_priority, 5);
            rt_thread_startup(thread);//可能不需要
		}
		else
		{
			return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
		}

		return pdPASS;
    }
#endif /* configSUPPORT_STATIC_ALLOCATION */





/** @cond */
/**
 * xTaskCreateRestricted() should only be used in systems that include an MPU
 * implementation.
 *
 * Create a new task and add it to the list of tasks that are ready to run.
 * The function parameters define the memory regions and associated access
 * permissions allocated to the task.
 *
 * @param pxTaskDefinition Pointer to a structure that contains a member
 * for each of the normal xTaskCreate() parameters (see the xTaskCreate() API
 * documentation) plus an optional stack buffer and the memory region
 * definitions.
 *
 * @param pxCreatedTask Used to pass back a handle by which the created task
 * can be referenced.
 *
 * @return pdPASS if the task was successfully created and added to a ready
 * list, otherwise an error code defined in the file projdefs.h
 *
 * Example usage:
 * @code{c}
 * // Create an TaskParameters_t structure that defines the task to be created.
 * static const TaskParameters_t xCheckTaskParameters =
 * {
 * 	vATask,		// pvTaskCode - the function that implements the task.
 * 	"ATask",	// pcName - just a text name for the task to assist debugging.
 * 	100,		// usStackDepth	- the stack size DEFINED IN BYTES.
 * 	NULL,		// pvParameters - passed into the task function as the function parameters.
 * 	( 1UL | portPRIVILEGE_BIT ),// uxPriority - task priority, set the portPRIVILEGE_BIT if the task should run in a privileged state.
 * 	cStackBuffer,// puxStackBuffer - the buffer to be used as the task stack.
 *
 * 	// xRegions - Allocate up to three separate memory regions for access by
 * 	// the task, with appropriate access permissions.  Different processors have
 * 	// different memory alignment requirements - refer to the FreeRTOS documentation
 * 	// for full information.
 * 	{
 * 		// Base address					Length	Parameters
 *         { cReadWriteArray,				32,		portMPU_REGION_READ_WRITE },
 *         { cReadOnlyArray,				32,		portMPU_REGION_READ_ONLY },
 *         { cPrivilegedOnlyAccessArray,	128,	portMPU_REGION_PRIVILEGED_READ_WRITE }
 * 	}
 * };
 *
 * int main( void )
 * {
 * TaskHandle_t xHandle;
 *
 * 	// Create a task from the const structure defined above.  The task handle
 * 	// is requested (the second parameter is not NULL) but in this case just for
 * 	// demonstration purposes as its not actually used.
 * 	xTaskCreateRestricted( &xRegTest1Parameters, &xHandle );
 *
 * 	// Start the scheduler.
 * 	vTaskStartScheduler();
 *
 * 	// Will only get here if there was insufficient memory to create the idle
 * 	// and/or timer task.
 * 	for( ;; );
 * }
 * @endcode
 * \ingroup Tasks
 */
#if( portUSING_MPU_WRAPPERS == 1 )
	BaseType_t xTaskCreateRestricted( const TaskParameters_t * const pxTaskDefinition, TaskHandle_t *pxCreatedTask )
    {
        rt_kprintf("error : xTaskCreateRestricted not implemented!!!\n\n");
        return pdPASS;
    }
#endif


/**
 * Memory regions are assigned to a restricted task when the task is created by
 * a call to xTaskCreateRestricted().  These regions can be redefined using
 * vTaskAllocateMPURegions().
 *
 * @param xTask The handle of the task being updated.
 *
 * @param xRegions A pointer to an MemoryRegion_t structure that contains the
 * new memory region definitions.
 *
 * Example usage:
 *
 * @code{c}
 * // Define an array of MemoryRegion_t structures that configures an MPU region
 * // allowing read/write access for 1024 bytes starting at the beginning of the
 * // ucOneKByte array.  The other two of the maximum 3 definable regions are
 * // unused so set to zero.
 * static const MemoryRegion_t xAltRegions[ portNUM_CONFIGURABLE_REGIONS ] =
 * {
 * 	// Base address		Length		Parameters
 * 	{ ucOneKByte,		1024,		portMPU_REGION_READ_WRITE },
 * 	{ 0,				0,			0 },
 * 	{ 0,				0,			0 }
 * };
 *
 * void vATask( void *pvParameters )
 * {
 * 	// This task was created such that it has access to certain regions of
 * 	// memory as defined by the MPU configuration.  At some point it is
 * 	// desired that these MPU regions are replaced with that defined in the
 * 	// xAltRegions const struct above.  Use a call to vTaskAllocateMPURegions()
 * 	// for this purpose.  NULL is used as the task handle to indicate that this
 * 	// function should modify the MPU regions of the calling task.
 * 	vTaskAllocateMPURegions( NULL, xAltRegions );
 *
 * 	// Now the task can continue its function, but from this point on can only
 * 	// access its stack and the ucOneKByte array (unless any other statically
 * 	// defined or shared regions have been declared elsewhere).
 * }
 * @endcode
 * \ingroup Tasks
 */
void vTaskAllocateMPURegions( TaskHandle_t xTask, const MemoryRegion_t * const pxRegions )
{
    rt_kprintf("error : vTaskAllocateMPURegions not implemented!!!\n\n");
    return ;
}

/** @endcond */

/**
 * 从RTOS实时内核的管理中删除任务。
 *
 *
 * 必须将INCLUDE_vTaskDelete定义为1才能使用此功能。
 *
 * @note The idle task is responsible for freeing the kernel allocated
 * memory from tasks that have been deleted.  It is therefore important that
 * the idle task is not starved of microcontroller processing time if your
 * application makes any calls to vTaskDelete ().  Memory allocated by the
 * task code is not automatically freed, and should be freed before the task
 * is deleted.
 *
 * See the demo application file death.c for sample code that utilises
 * vTaskDelete ().
 *
 * @param xTaskToDelete 要删除的任务句柄。 传递NULL将导致调用任务被删除。
 *
 * Example usage:
 * @code{c}
 *  void vOtherFunction( void )
 *  {
 *  TaskHandle_t xHandle;
 *
 * 	 // Create the task, storing the handle.
 * 	 xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 * 	 // Use the handle to delete the task.
 * 	 vTaskDelete( xHandle );
 *  }
 * @endcode
 * \ingroup Tasks
 */
#if ( INCLUDE_vTaskDelete == 1 )
    void vTaskDelete( TaskHandle_t xTaskToDelete )
    {
    // #if( configSUPPORT_STATIC_ALLOCATION == 1 )
        if(NULL == xTaskToDelete)
        {
            // 对于传入参数为空 即删除自身线程的
            // 这里暂时只能采用无限延时来实现
            while(1)
            {
                rt_thread_delay(1000);
            }
            rt_kprintf("delete allocated thread [%s]\n", rt_thread_self()->name);
            rt_thread_delete(rt_thread_self());//动态删除
        }
        else
        {
            rt_kprintf("delete allocated thread [%s]\n", ((rt_thread_t)xTaskToDelete)->name);
            rt_thread_delete((rt_thread_t)xTaskToDelete);//动态删除
        }
    // #else
    //     if(NULL == xTaskToDelete)
    //     {
    //         rt_kprintf("delete static thread [%s]\n", rt_thread_self()->name);
    //         rt_thread_detach(rt_thread_self());//静态删除
    //     }
    //     else
    //     {
    //         rt_kprintf("delete static thread [%s]\n", ((rt_thread_t)xTaskToDelete)->name);
    //         rt_thread_detach((rt_thread_t)xTaskToDelete);//静态删除
    //     }
    // #endif
    }
#endif /* INCLUDE_vTaskDelete */

/*-----------------------------------------------------------
 * TASK CONTROL API
 *----------------------------------------------------------*/

/**
 * Delay a task for a given number of ticks.
 *
 * The actual time that the task remains blocked depends on the tick rate.
 * The constant portTICK_PERIOD_MS can be used to calculate real time from
 * the tick rate - with the resolution of one tick period.
 *
 * INCLUDE_vTaskDelay must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * vTaskDelay() specifies a time at which the task wishes to unblock relative to
 * the time at which vTaskDelay() is called.  For example, specifying a block
 * period of 100 ticks will cause the task to unblock 100 ticks after
 * vTaskDelay() is called.  vTaskDelay() does not therefore provide a good method
 * of controlling the frequency of a periodic task as the path taken through the
 * code, as well as other task and interrupt activity, will effect the frequency
 * at which vTaskDelay() gets called and therefore the time at which the task
 * next executes.  See vTaskDelayUntil() for an alternative API function designed
 * to facilitate fixed frequency execution.  It does this by specifying an
 * absolute time (rather than a relative time) at which the calling task should
 * unblock.
 *
 * @param xTicksToDelay The amount of time, in tick periods, that
 * the calling task should block.
 *
 * Example usage:
 * @code{c}
 *  void vTaskFunction( void * pvParameters )
 *  {
 *  // Block for 500ms.
 *  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
 *
 * 	 for( ;; )
 * 	 {
 * 		 // Simply toggle the LED every 500ms, blocking between each toggle.
 * 		 vToggleLED();
 * 		 vTaskDelay( xDelay );
 * 	 }
 *  }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_vTaskDelay == 1 )
	void vTaskDelay( const TickType_t xTicksToDelay )
	{
        rt_thread_delay((rt_tick_t)xTicksToDelay);
    }
#endif /* INCLUDE_vTaskDelay */

/**
 * 延时任务到指定时间
 *
 * INCLUDE_vTaskDelayUntil must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * 这个功能可以被周期性任务使用，以保证恒定的执行频率。
 *
 * 这个函数与vTaskDelay()有一个重要的不同之处：vTaskDelay()将使一个任务从调用vTaskDelay()的时候开始阻塞指定的ticks数。
 * 因此，使用vTaskDelay()本身很难产生一个固定的执行频率，因为从一个任务开始执行到该任务调用vTaskDelay()之间的时间可能
 * 不是固定的[任务在调用之间可能会走不同的路径，或者每次执行时可能会被打断或被抢占不同的次数]。
 *
 * 而vTaskDelay()指定的是相对于函数被调用时的唤醒时间，vTaskDelayUntil()指定的是希望解封的绝对(准确)时间。
 *
 * 常量 portTICK_PERIOD_MS 可以用来计算实时的 tick rate - 分辨率为一个 tick
 *
 * @param pxPreviousWakeTime 指向一个变量的指针，该变量保存了任务最后一次解封的时间。
 * 在第一次使用之前，必须用当前时间来初始化该变量（见下面的例子）。 在此之后，变量会在vTaskDelayUntil（）中自动更新。
 *
 * @param xTimeIncrement 循环时间段。 任务将在时间*pxPreviousWakeTime+xTimeIncrement时解锁。
 * 用相同的xTimeIncrement参数值调用vTaskDelayUntil将导致任务以固定的接口周期执行。
 *
 * Example usage:
 * @code{c}
 *  // Perform an action every 10 ticks.
 *  void vTaskFunction( void * pvParameters )
 *  {
 *  TickType_t xLastWakeTime;
 *  const TickType_t xFrequency = 10;
 *
 * 	 // Initialise the xLastWakeTime variable with the current time.
 * 	 xLastWakeTime = xTaskGetTickCount ();
 * 	 for( ;; )
 * 	 {
 * 		 // Wait for the next cycle.
 * 		 vTaskDelayUntil( &xLastWakeTime, xFrequency );
 *
 * 		 // Perform action here.
 * 	 }
 *  }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_vTaskDelayUntil == 1 )
    void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement )
	{
        rt_thread_delay((rt_tick_t)xTimeIncrement);
    }
#endif /* INCLUDE_vTaskDelayUntil */


/**
 * Obtain the priority of any task.
 *
 * INCLUDE_uxTaskPriorityGet must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * @param xTask Handle of the task to be queried.  Passing a NULL
 * handle results in the priority of the calling task being returned.
 *
 * @return The priority of xTask.
 *
 * Example usage:
 * @code{c}
 *  void vAFunction( void )
 *  {
 *  TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to obtain the priority of the created task.
 *   // It was created with tskIDLE_PRIORITY, but may have changed
 *   // it itself.
 *   if( uxTaskPriorityGet( xHandle ) != tskIDLE_PRIORITY )
 *   {
 *       // The task has changed it's priority.
 *   }
 *
 *   // ...
 *
 *   // Is our priority higher than the created task?
 *   if( uxTaskPriorityGet( xHandle ) < uxTaskPriorityGet( NULL ) )
 *   {
 *       // Our priority (obtained using NULL handle) is higher.
 *   }
 * }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_uxTaskPriorityGet == 1 )
	UBaseType_t uxTaskPriorityGet( TaskHandle_t xTask )
	{
	    UBaseType_t uxReturn;
        rt_thread_t thread = RT_NULL;


        if(NULL == xTask)
        {
            thread = rt_thread_self();
            return thread->current_priority;
        }else
        {
            return ((rt_thread_t)xTask)->current_priority;
        }
	}
#endif /* INCLUDE_uxTaskPriorityGet */


/**
 * A version of uxTaskPriorityGet() that can be used from an ISR.
 *
 * @param xTask Handle of the task to be queried.  Passing a NULL
 * handle results in the priority of the calling task being returned.
 *
 * @return The priority of xTask.
 *
 */
#if ( INCLUDE_uxTaskPriorityGet == 1 )
	UBaseType_t uxTaskPriorityGetFromISR( TaskHandle_t xTask )
	{
        return uxTaskPriorityGet(xTask);
	}
#endif /* INCLUDE_uxTaskPriorityGet */


/**
 * 获得任何任务的状态。
 *
 * 状态由eTaskState枚举类型编码。
 *
 * INCLUDE_eTaskGetState must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * @param xTask 要查询的任务的处理方式。
 *
 * @return 函数被调用时xTask的状态。 注意在函数被调用和函数返回值被调用任务测试之间，任务的状态可能会发生变化。
 */


#if ( INCLUDE_eTaskGetState == 1 )
	eTaskState eTaskGetState( TaskHandle_t xTask )
    {
        switch (((rt_thread_t)xTask)->stat)
        {
            case RT_THREAD_RUNNING: return eRunning;
            case RT_THREAD_READY: return eReady;
            // case RT_THREAD_BLOCK: return eBlocked;//RT_THREAD_BLOCK == RT_THREAD_SUSPEND
            case RT_THREAD_SUSPEND: return eSuspended;
            case RT_THREAD_CLOSE: return eDeleted;
            default: return 0xff;//RT_THREAD_INIT
        }
    }
#endif /* INCLUDE_eTaskGetState */


/**
 * Set the priority of any task.
 *
 * INCLUDE_vTaskPrioritySet must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * A context switch will occur before the function returns if the priority
 * being set is higher than the currently executing task.
 *
 * @param xTask Handle to the task for which the priority is being set.
 * Passing a NULL handle results in the priority of the calling task being set.
 *
 * @param uxNewPriority The priority to which the task will be set.
 *
 * Example usage:
 * @code{c}
 *  void vAFunction( void )
 *  {
 *  TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to raise the priority of the created task.
 *   vTaskPrioritySet( xHandle, tskIDLE_PRIORITY + 1 );
 *
 *   // ...
 *
 *   // Use a NULL handle to raise our priority to the same value.
 *   vTaskPrioritySet( NULL, tskIDLE_PRIORITY + 1 );
 *  }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_vTaskPrioritySet == 1 )
	void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority )
	{
        rt_thread_control((rt_thread_t)xTask, RT_THREAD_CTRL_CHANGE_PRIORITY, (rt_uint8_t *)&uxNewPriority);
	}
#endif /* INCLUDE_vTaskPrioritySet */


/**
 * Suspend a task.
 *
 * INCLUDE_vTaskSuspend must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * When suspended, a task will never get any microcontroller processing time,
 * no matter what its priority.
 *
 * Calls to vTaskSuspend are not accumulative -
 * i.e. calling vTaskSuspend () twice on the same task still only requires one
 * call to vTaskResume () to ready the suspended task.
 *
 * @param xTaskToSuspend Handle to the task being suspended.  Passing a NULL
 * handle will cause the calling task to be suspended.
 *
 * Example usage:
 * @code{c}
 *  void vAFunction( void )
 *  {
 *  TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to suspend the created task.
 *   vTaskSuspend( xHandle );
 *
 *   // ...
 *
 *   // The created task will not run during this period, unless
 *   // another task calls vTaskResume( xHandle ).
 *
 *   //...
 *
 *
 *   // Suspend ourselves.
 *   vTaskSuspend( NULL );
 *
 *   // We cannot get here unless another task calls vTaskResume
 *   // with our handle as the parameter.
 *  }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_vTaskSuspend == 1 )
	void vTaskSuspend( TaskHandle_t xTaskToSuspend )
	{
        rt_thread_t thread;
        if(NULL == xTaskToSuspend)
        {
            thread = rt_thread_self();
            rt_thread_suspend(thread);
        }else
        {
            rt_thread_suspend((rt_thread_t)xTaskToSuspend);
        }
        rt_schedule();
    }
#endif /* INCLUDE_vTaskSuspend */





/**
 * Resumes a suspended task.
 *
 * INCLUDE_vTaskSuspend must be defined as 1 for this function to be available.
 * See the configuration section for more information.
 *
 * A task that has been suspended by one or more calls to vTaskSuspend ()
 * will be made available for running again by a single call to
 * vTaskResume ().
 *
 * @param xTaskToResume Handle to the task being readied.
 *
 * Example usage:
 * @code{c}
 *  void vAFunction( void )
 *  {
 *  TaskHandle_t xHandle;
 *
 *   // Create a task, storing the handle.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
 *
 *   // ...
 *
 *   // Use the handle to suspend the created task.
 *   vTaskSuspend( xHandle );
 *
 *   // ...
 *
 *   // The created task will not run during this period, unless
 *   // another task calls vTaskResume( xHandle ).
 *
 *   //...
 *
 *
 *   // Resume the suspended task ourselves.
 *   vTaskResume( xHandle );
 *
 *   // The created task will once again get microcontroller processing
 *   // time in accordance with its priority within the system.
 *  }
 * @endcode
 * \ingroup TaskCtrl
 */
#if ( INCLUDE_vTaskSuspend == 1 )
	void vTaskResume( TaskHandle_t xTaskToResume )
	{
        rt_thread_resume((rt_thread_t)xTaskToResume);
    }
#endif /* INCLUDE_vTaskSuspend */



/**
 * An implementation of vTaskResume() that can be called from within an ISR.
 *
 * INCLUDE_xTaskResumeFromISR must be defined as 1 for this function to be
 * available.  See the configuration section for more information.
 *
 * A task that has been suspended by one or more calls to vTaskSuspend ()
 * will be made available for running again by a single call to
 * xTaskResumeFromISR ().
 *
 * xTaskResumeFromISR() should not be used to synchronise a task with an
 * interrupt if there is a chance that the interrupt could arrive prior to the
 * task being suspended - as this can lead to interrupts being missed. Use of a
 * semaphore as a synchronisation mechanism would avoid this eventuality.
 *
 * @param xTaskToResume Handle to the task being readied.
 *
 * @return pdTRUE if resuming the task should result in a context switch,
 * otherwise pdFALSE. This is used by the ISR to determine if a context switch
 * may be required following the ISR.
 *
 * \ingroup TaskCtrl
 */
#if ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) )
	BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume )
	{
        if(-RT_ERROR == rt_thread_resume((rt_thread_t)xTaskToResume))
            return pdFALSE;
        else
            return pdTRUE;
    }
#endif /* ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) ) */



/*-----------------------------------------------------------
 * SCHEDULER CONTROL
 *----------------------------------------------------------*/
/** @cond */
/**
 * Starts the real time kernel tick processing.
 *
 * After calling the kernel has control over which tasks are executed and when.
 *
 * See the demo application file main.c for an example of creating
 * tasks and starting the kernel.
 *
 * Example usage:
 * @code{c}
 *  void vAFunction( void )
 *  {
 *   // Create at least one task before starting the kernel.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
 *
 *   // Start the real time kernel with preemption.
 *   vTaskStartScheduler ();
 *
 *   // Will not get here unless a task calls vTaskEndScheduler ()
 *  }
 * @endcode
 *
 * \ingroup SchedulerControl
 */
void vTaskStartScheduler( void )
{
    extern int rtthread_startup(void);

    // 先关全局中断
    rt_hw_interrupt_disable();

    // 必须手动置位 否则 newlib 中会报错
    xSchedulerRunning = pdTRUE;

    rtthread_startup();
//     rt_system_scheduler_start();
}

/**
 * Stops the real time kernel tick.
 *
 * @note At the time of writing only the x86 real mode port, which runs on a PC
 * in place of DOS, implements this function.
 *
 * All created tasks will be automatically deleted and multitasking
 * (either preemptive or cooperative) will stop.
 * Execution then resumes from the point where vTaskStartScheduler ()
 * was called, as if vTaskStartScheduler () had just returned.
 *
 * See the demo application file main. c in the demo/PC directory for an
 * example that uses vTaskEndScheduler ().
 *
 * vTaskEndScheduler () requires an exit function to be defined within the
 * portable layer (see vPortEndScheduler () in port. c for the PC port).  This
 * performs hardware specific operations such as stopping the kernel tick.
 *
 * vTaskEndScheduler () will cause all of the resources allocated by the
 * kernel to be freed - but will not free resources allocated by application
 * tasks.
 *
 * Example usage:
 * @code{c}
 *  void vTaskCode( void * pvParameters )
 *  {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *
 *       // At some point we want to end the real time kernel processing
 *       // so call ...
 *       vTaskEndScheduler ();
 *   }
 *  }
 *
 *  void vAFunction( void )
 *  {
 *   // Create at least one task before starting the kernel.
 *   xTaskCreate( vTaskCode, "NAME", STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
 *
 *   // Start the real time kernel with preemption.
 *   vTaskStartScheduler ();
 *
 *   // Will only get here when the vTaskCode () task has called
 *   // vTaskEndScheduler ().  When we get here we are back to single task
 *   // execution.
 *  }
 * @endcode
 * \ingroup SchedulerControl
 */
void vTaskEndScheduler( void )
{
	/* Stop the scheduler interrupts and call the portable scheduler end
	routine so the original ISRs can be restored if necessary.  The port
	layer must ensure interrupts enable	bit is left in the correct state. */
	portDISABLE_INTERRUPTS();
	xSchedulerRunning = pdFALSE;
	vPortEndScheduler();
}

#if ( configUSE_NEWLIB_REENTRANT == 1 )
//Return global reent struct if FreeRTOS isn't running,
struct _reent* __getreent() {
	//No lock needed because if this changes, we won't be running anymore.
	// TCB_t *currTask=xTaskGetCurrentTaskHandle();

    rt_thread_t currTask=xTaskGetCurrentTaskHandle();
	if (currTask==NULL) {
		//No task running. Return global struct.
		return _GLOBAL_REENT;
	} else {
		//We have a task; return its reentrant struct.
		return &currTask->xNewLib_reent;
	}
}
#endif

/** @endcond */

/**
 * 暂停调度器而不禁用中断。
 *
 * Context switches will not occur while the scheduler is suspended.
 *
 * After calling vTaskSuspendAll () the calling task will continue to execute
 * without risk of being swapped out until a call to xTaskResumeAll () has been
 * made.
 *
 * API functions that have the potential to cause a context switch (for example,
 * vTaskDelayUntil(), xQueueSend(), etc.) must not be called while the scheduler
 * is suspended.
 *
 * Example usage:
 * @code{c}
 *  void vTask1( void * pvParameters )
 *  {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *
 *       // ...
 *
 *       // At some point the task wants to perform a long operation during
 *       // which it does not want to get swapped out.  It cannot use
 *       // taskENTER_CRITICAL ()/taskEXIT_CRITICAL () as the length of the
 *       // operation may cause interrupts to be missed - including the
 *       // ticks.
 *
 *       // Prevent the real time kernel swapping out the task.
 *       vTaskSuspendAll ();
 *
 *       // Perform the operation here.  There is no need to use critical
 *       // sections as we have all the microcontroller processing time.
 *       // During this time interrupts will still operate and the kernel
 *       // tick count will be maintained.
 *
 *       // ...
 *
 *       // The operation is complete.  Restart the kernel.
 *       xTaskResumeAll ();
 *   }
 *  }
 * @endcode
 * \ingroup SchedulerControl
 */
void vTaskSuspendAll( void )
{
	/* A critical section is not required as the variable is of type
	BaseType_t.  Please read Richard Barry's reply in the following link to a
	post in the FreeRTOS support forum before reporting this as a bug! -
	http://goo.gl/wu4acr */
    rt_enter_critical();
}

/**
 * Resumes scheduler activity after it was suspended by a call to
 * vTaskSuspendAll().
 *
 * xTaskResumeAll() only resumes the scheduler.  It does not unsuspend tasks
 * that were previously suspended by a call to vTaskSuspend().
 *
 * @return If resuming the scheduler caused a context switch then pdTRUE is
 *		  returned, otherwise pdFALSE is returned.
 *
 * Example usage:
 * @code{c}
 *  void vTask1( void * pvParameters )
 *  {
 *   for( ;; )
 *   {
 *       // Task code goes here.
 *
 *       // ...
 *
 *       // At some point the task wants to perform a long operation during
 *       // which it does not want to get swapped out.  It cannot use
 *       // taskENTER_CRITICAL ()/taskEXIT_CRITICAL () as the length of the
 *       // operation may cause interrupts to be missed - including the
 *       // ticks.
 *
 *       // Prevent the real time kernel swapping out the task.
 *       vTaskSuspendAll ();
 *
 *       // Perform the operation here.  There is no need to use critical
 *       // sections as we have all the microcontroller processing time.
 *       // During this time interrupts will still operate and the real
 *       // time kernel tick count will be maintained.
 *
 *       // ...
 *
 *       // The operation is complete.  Restart the kernel.  We want to force
 *       // a context switch - but there is no point if resuming the scheduler
 *       // caused a context switch already.
 *       if( !xTaskResumeAll () )
 *       {
 *            taskYIELD ();
 *       }
 *   }
 *  }
 * @endcode
 * \ingroup SchedulerControl
 */
BaseType_t xTaskResumeAll( void )
{
    rt_exit_critical();
    return pdTRUE;
}

/*-----------------------------------------------------------
 * TASK UTILITIES
 *----------------------------------------------------------*/

/**
 * Get tick count
 *
 * @return The count of ticks since vTaskStartScheduler was called.
 *
 * \ingroup TaskUtils
 */
TickType_t xTaskGetTickCount( void )
{
	return rt_tick_get();
}

/**
 * Get tick count from ISR
 *
 * @return The count of ticks since vTaskStartScheduler was called.
 *
 * This is a version of xTaskGetTickCount() that is safe to be called from an
 * ISR - provided that TickType_t is the natural word size of the
 * microcontroller being used or interrupt nesting is either not supported or
 * not being used.
 *
 * \ingroup TaskUtils
 */
TickType_t xTaskGetTickCountFromISR( void )
{
	return rt_tick_get();
}

/**
 * 获取当前任务数量
 *
 * @return 实时内核当前管理的任务数量。这包括所有准备好的、阻塞的和暂停的任务。
 * 已被删除但尚未被空闲任务释放的任务也将包含在计数中。
 * The number of tasks that the real time kernel is currently managing.
 * This includes all ready, blocked and suspended tasks.  A task that
 * has been deleted but not yet freed by the idle task will also be
 * included in the count.
 *
 * \ingroup TaskUtils
 */
UBaseType_t uxTaskGetNumberOfTasks( void )
{
	/* A critical section is not required because the variables are of type
	BaseType_t. */
	//利用rtt容器(object.c)可以实现，这里暂不实现
    rt_kprintf("error : uxTaskGetNumberOfTasks not implemented!!!\n\n");
    return NULL;
}

/**
 * Get task name
 *
 * @return The text (human readable) name of the task referenced by the handle
 * xTaskToQuery.  A task can query its own name by either passing in its own
 * handle, or by setting xTaskToQuery to NULL.  INCLUDE_pcTaskGetTaskName must be
 * set to 1 in FreeRTOSConfig.h for pcTaskGetTaskName() to be available.
 *
 * \ingroup TaskUtils
 */
#if ( INCLUDE_pcTaskGetTaskName == 1 )
	char *pcTaskGetTaskName( TaskHandle_t xTaskToQuery ) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	{
        rt_thread_t thread = RT_NULL;
        if(NULL == xTaskToQuery)
        {
            thread = rt_thread_self();
        }else
        {
            thread = (rt_thread_t)xTaskToQuery;
        }
        return &(thread->name[0]);
	}

#endif /* INCLUDE_pcTaskGetTaskName */


/**
 * Returns the high water mark of the stack associated with xTask.
 *
 * INCLUDE_uxTaskGetStackHighWaterMark must be set to 1 in FreeRTOSConfig.h for
 * this function to be available.
 *
 * High water mark is the minimum free stack space there has been (in bytes
 * rather than words as found in vanilla FreeRTOS) since the task started.
 * The smaller the returned number the closer the task has come to overflowing its stack.
 *
 * @param xTask Handle of the task associated with the stack to be checked.
 * Set xTask to NULL to check the stack of the calling task.
 *
 * @return The smallest amount of free stack space there has been (in bytes
 * rather than words as found in vanilla FreeRTOS) since the task referenced by
 * xTask was created.
 */
#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

	static uint32_t prvTaskCheckFreeStackSpace( const uint8_t * pucStackByte )
	{
	    uint32_t ulCount = 0U;

		while( *pucStackByte == ( uint8_t ) tskSTACK_FILL_BYTE )
		{
			pucStackByte -= portSTACK_GROWTH;
			ulCount++;
		}

		ulCount /= ( uint32_t ) sizeof( StackType_t ); /*lint !e961 Casting is not redundant on smaller architectures. */

		return ( uint32_t ) ulCount;
	}

#endif /* ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) ) */
#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )
	UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask )
	{
        rt_thread_t thread = RT_NULL;
        if(NULL == xTask)
        {
            thread = rt_thread_self();
        }else
        {
            thread = (rt_thread_t)xTask;
        }
        // return thread->sp;
        return ( UBaseType_t ) prvTaskCheckFreeStackSpace( thread->sp );
	}
#endif /* INCLUDE_uxTaskGetStackHighWaterMark */



/**
 * Returns the start of the stack associated with xTask.
 *
 * INCLUDE_pxTaskGetStackStart must be set to 1 in FreeRTOSConfig.h for
 * this function to be available.
 *
 * Returns the highest stack memory address on architectures where the stack grows down
 * from high memory, and the lowest memory address on architectures where the
 * stack grows up from low memory.
 *
 * @param xTask Handle of the task associated with the stack returned.
 * Set xTask to NULL to return the stack of the calling task.
 *
 * @return A pointer to the start of the stack.
 */
#if (INCLUDE_pxTaskGetStackStart == 1)

	uint8_t* pxTaskGetStackStart( TaskHandle_t xTask)
	{
		rt_thread_t thread = RT_NULL;
        if(NULL == xTask)
        {
            thread = rt_thread_self();
        }else
        {
            thread = (rt_thread_t)xTask;
        }
        return thread->sp;
	}

#endif /* INCLUDE_pxTaskGetStackStart */

/* 当使用trace宏时，有时需要在FreeRTOS.h之前加入task.h，当这样做的时候，askHookFunction_t还没有被定义，
所以以下两个原型会导致编译错误。 这个问题可以通过简单地防止包含这两个原型来解决，
除非configUSE_APPLICATION_TASK_TAG配置常量明确要求这两个原型。

When using trace macros it is sometimes necessary to include task.h before
FreeRTOS.h.  When this is done TaskHookFunction_t will not yet have been defined,
so the following two prototypes will cause a compilation error.  This can be
fixed by simply guarding against the inclusion of these two prototypes unless
they are explicitly required by the configUSE_APPLICATION_TASK_TAG configuration
constant. */
#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
		/**
		 * Sets pxHookFunction to be the task hook function used by the task xTask.
		 * @param xTask Handle of the task to set the hook function for
		 *              Passing xTask as NULL has the effect of setting the calling
		 *              tasks hook function.
		 * @param pxHookFunction  Pointer to the hook function.
		 */
		void vTaskSetApplicationTaskTag( TaskHandle_t xTask, TaskHookFunction_t pxHookFunction )
        {
            //暂时啥也不干
            rt_kprintf("error : vTaskSetApplicationTaskTag not implemented!!!\n\n");
        }

		/**
		 * Get the hook function assigned to given task.
		 * @param xTask Handle of the task to get the hook function for
		 *              Passing xTask as NULL has the effect of getting the calling
		 *              tasks hook function.
		 * @return The pxHookFunction value assigned to the task xTask.
		 */
		TaskHookFunction_t xTaskGetApplicationTaskTag( TaskHandle_t xTask )
        {
            //暂时啥也不干
            rt_kprintf("error : xTaskGetApplicationTaskTag not implemented!!!\n\n");
        }
	#endif /* configUSE_APPLICATION_TASK_TAG ==1 */
#endif /* ifdef configUSE_APPLICATION_TASK_TAG */
#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )

	/**
	 * Set local storage pointer specific to the given task.
	 *
	 * Each task contains an array of pointers that is dimensioned by the
	 * configNUM_THREAD_LOCAL_STORAGE_POINTERS setting in FreeRTOSConfig.h.
	 * The kernel does not use the pointers itself, so the application writer
	 * can use the pointers for any purpose they wish.
	 *
	 * @param xTaskToSet  Task to set thread local storage pointer for
	 * @param xIndex The index of the pointer to set, from 0 to
	 *               configNUM_THREAD_LOCAL_STORAGE_POINTERS - 1.
	 * @param pvValue  Pointer value to set.
	 */
	void vTaskSetThreadLocalStoragePointer( TaskHandle_t xTaskToSet, BaseType_t xIndex, void *pvValue )
    {
        // 该部分是实现pthread中线程中用户申请变量垃圾回收的一些函数，暂时先不实现 可能会造成内存泄漏
        // 暂时啥也不干
        // rt_kprintf("error : vTaskSetThreadLocalStoragePointer not implemented!!!\n\n");
    }


	/**
	 * Get local storage pointer specific to the given task.
	 *
	 * Each task contains an array of pointers that is dimensioned by the
	 * configNUM_THREAD_LOCAL_STORAGE_POINTERS setting in FreeRTOSConfig.h.
	 * The kernel does not use the pointers itself, so the application writer
	 * can use the pointers for any purpose they wish.
	 *
	 * @param xTaskToQuery  Task to get thread local storage pointer for
	 * @param xIndex The index of the pointer to get, from 0 to
	 *               configNUM_THREAD_LOCAL_STORAGE_POINTERS - 1.
	 * @return  Pointer value
	 */
	void *pvTaskGetThreadLocalStoragePointer( TaskHandle_t xTaskToQuery, BaseType_t xIndex )
    {
        // 该部分是实现pthread中线程中用户申请变量垃圾回收的一些函数，暂时先不实现 可能会造成内存泄漏
        //暂时啥也不干 可能有bug
        // rt_kprintf("error : pvTaskGetThreadLocalStoragePointer not implemented!!!\n\n");
        return NULL;
    }

	#if ( configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS )

		/**
		 * Prototype of local storage pointer deletion callback.
		 */
		typedef void (*TlsDeleteCallbackFunction_t)( int, void * );

		/**
		 * Set local storage pointer and deletion callback.
		 *
		 * Each task contains an array of pointers that is dimensioned by the
		 * configNUM_THREAD_LOCAL_STORAGE_POINTERS setting in FreeRTOSConfig.h.
		 * The kernel does not use the pointers itself, so the application writer
		 * can use the pointers for any purpose they wish.
		 *
		 * Local storage pointers set for a task can reference dynamically
		 * allocated resources. This function is similar to
		 * vTaskSetThreadLocalStoragePointer, but provides a way to release
		 * these resources when the task gets deleted. For each pointer,
		 * a callback function can be set. This function will be called
		 * when task is deleted, with the local storage pointer index
		 * and value as arguments.
		 *
		 * @param xTaskToSet  Task to set thread local storage pointer for
		 * @param xIndex The index of the pointer to set, from 0 to
		 *               configNUM_THREAD_LOCAL_STORAGE_POINTERS - 1.
		 * @param pvValue  Pointer value to set.
		 * @param pvDelCallback  Function to call to dispose of the local
		 *                       storage pointer when the task is deleted.
		 */
		void vTaskSetThreadLocalStoragePointerAndDelCallback( TaskHandle_t xTaskToSet, BaseType_t xIndex, void *pvValue, TlsDeleteCallbackFunction_t pvDelCallback)
        {
            // 该部分是实现pthread中线程中用户申请变量垃圾回收的一些函数，暂时先不实现 可能会造成内存泄漏
            //暂时啥也不干
            // rt_kprintf("error : vTaskSetThreadLocalStoragePointerAndDelCallback not implemented!!!\n\n");
        }
	#endif

#endif

/**
 * Calls the hook function associated with xTask. Passing xTask as NULL has
 * the effect of calling the Running tasks (the calling task) hook function.
 *
 * @param xTask  Handle of the task to call the hook for.
 * @param pvParameter  Parameter passed to the hook function for the task to interpret as it
 * wants.  The return value is the value returned by the task hook function
 * registered by the user.
 */
#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter )
	{
        //暂时啥也不干
        rt_kprintf("error : xTaskCallApplicationTaskHook not implemented!!!\n\n");
	}
#endif /* configUSE_APPLICATION_TASK_TAG */


/**
 * 获取当前CPU的空闲任务句柄
 *
 * xTaskGetIdleTaskHandle() is only available if
 * INCLUDE_xTaskGetIdleTaskHandle is set to 1 in FreeRTOSConfig.h.
 *
 * @return The handle of the idle task.  It is not valid to call
 * xTaskGetIdleTaskHandle() before the scheduler has been started.
 */
#if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )

	TaskHandle_t xTaskGetIdleTaskHandle( void )
	{
		/* If xTaskGetIdleTaskHandle() is called before the scheduler has been
		started, then xIdleTaskHandle will be NULL. */
        return rt_thread_idle_gethandler();
	}

	TaskHandle_t xTaskGetIdleTaskHandleForCPU( UBaseType_t cpuid )
	{
	    /* If xTaskGetIdleTaskHandleForCPU() is called before the scheduler has been
        started, then xIdleTaskHandle will be NULL. */
        return rt_thread_idle_gethandler();
	}

#endif /* INCLUDE_xTaskGetIdleTaskHandle */


/**
 * 获取给定CPU的空闲任务的句柄
 *
 * xTaskGetIdleTaskHandleForCPU() is only available if
 * INCLUDE_xTaskGetIdleTaskHandle is set to 1 in FreeRTOSConfig.h.
 *
 * @param cpuid The CPU to get the handle for
 *
 * @return Idle task handle of a given cpu. It is not valid to call
 * xTaskGetIdleTaskHandleForCPU() before the scheduler has been started.
 */
// TaskHandle_t xTaskGetIdleTaskHandleForCPU( UBaseType_t cpuid );//实现在上面




/**
 * Get the state of tasks in the system.
 *
 * configUSE_TRACE_FACILITY must be defined as 1 in FreeRTOSConfig.h for
 * uxTaskGetSystemState() to be available.
 *
 * uxTaskGetSystemState() populates an TaskStatus_t structure for each task in
 * the system.  TaskStatus_t structures contain, among other things, members
 * for the task handle, task name, task priority, task state, and total amount
 * of run time consumed by the task.  See the TaskStatus_t structure
 * definition in this file for the full member list.
 *
 * @note  This function is intended for debugging use only as its use results in
 * the scheduler remaining suspended for an extended period.
 *
 * @param pxTaskStatusArray A pointer to an array of TaskStatus_t structures.
 * The array must contain at least one TaskStatus_t structure for each task
 * that is under the control of the RTOS.  The number of tasks under the control
 * of the RTOS can be determined using the uxTaskGetNumberOfTasks() API function.
 *
 * @param uxArraySize The size of the array pointed to by the pxTaskStatusArray
 * parameter.  The size is specified as the number of indexes in the array, or
 * the number of TaskStatus_t structures contained in the array, not by the
 * number of bytes in the array.
 *
 * @param pulTotalRunTime If configGENERATE_RUN_TIME_STATS is set to 1 in
 * FreeRTOSConfig.h then *pulTotalRunTime is set by uxTaskGetSystemState() to the
 * total run time (as defined by the run time stats clock, see
 * http://www.freertos.org/rtos-run-time-stats.html) since the target booted.
 * pulTotalRunTime can be set to NULL to omit the total run time information.
 *
 * @return The number of TaskStatus_t structures that were populated by
 * uxTaskGetSystemState().  This should equal the number returned by the
 * uxTaskGetNumberOfTasks() API function, but will be zero if the value passed
 * in the uxArraySize parameter was too small.
 *
 * Example usage:
 * @code{c}
 * // This example demonstrates how a human readable table of run time stats
 * // information is generated from raw data provided by uxTaskGetSystemState().
 * // The human readable table is written to pcWriteBuffer
 * void vTaskGetRunTimeStats( char *pcWriteBuffer )
 * {
 * TaskStatus_t *pxTaskStatusArray;
 * volatile UBaseType_t uxArraySize, x;
 * uint32_t ulTotalRunTime, ulStatsAsPercentage;
 *
 *  // Make sure the write buffer does not contain a string.
 *  *pcWriteBuffer = 0x00;
 *
 *  // Take a snapshot of the number of tasks in case it changes while this
 *  // function is executing.
 *  uxArraySize = uxTaskGetNumberOfTasks();
 *
 *  // Allocate a TaskStatus_t structure for each task.  An array could be
 *  // allocated statically at compile time.
 *  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
 *
 *  if( pxTaskStatusArray != NULL )
 *  {
 *      // Generate raw status information about each task.
 *      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );
 *
 *      // For percentage calculations.
 *      ulTotalRunTime /= 100UL;
 *
 *      // Avoid divide by zero errors.
 *      if( ulTotalRunTime > 0 )
 *      {
 *          // For each populated position in the pxTaskStatusArray array,
 *          // format the raw data as human readable ASCII data
 *          for( x = 0; x < uxArraySize; x++ )
 *          {
 *              // What percentage of the total run time has the task used?
 *              // This will always be rounded down to the nearest integer.
 *              // ulTotalRunTimeDiv100 has already been divided by 100.
 *              ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;
 *
 *              if( ulStatsAsPercentage > 0UL )
 *              {
 *                  sprintf( pcWriteBuffer, "%s\t\t%lu\t\t%lu%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, ulStatsAsPercentage );
 *              }
 *              else
 *              {
 *                  // If the percentage is zero here then the task has
 *                  // consumed less than 1% of the total run time.
 *                  sprintf( pcWriteBuffer, "%s\t\t%lu\t\t<1%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
 *              }
 *
 *              pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
 *          }
 *      }
 *
 *      // The array is no longer needed, free the memory it consumes.
 *      vPortFree( pxTaskStatusArray );
 *  }
 * }
 * @endcode
 */
#if ( configUSE_TRACE_FACILITY == 1 )
	UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray, const UBaseType_t uxArraySize, uint32_t * const pulTotalRunTime )
	{
        //暂时啥也不干
        rt_kprintf("error : uxTaskGetSystemState not implemented!!!\n\n");
    }
#endif /* configUSE_TRACE_FACILITY */

/**
 * List all the current tasks.
 *
 * configUSE_TRACE_FACILITY and configUSE_STATS_FORMATTING_FUNCTIONS must
 * both be defined as 1 for this function to be available.  See the
 * configuration section of the FreeRTOS.org website for more information.
 *
 * @note This function will disable interrupts for its duration.  It is
 * not intended for normal application runtime use but as a debug aid.
 *
 * Lists all the current tasks, along with their current state and stack
 * usage high water mark.
 *
 * Tasks are reported as blocked ('B'), ready ('R'), deleted ('D') or
 * suspended ('S').
 *
 * @note This function is provided for convenience only, and is used by many of the
 * demo applications.  Do not consider it to be part of the scheduler.
 *
 * vTaskList() calls uxTaskGetSystemState(), then formats part of the
 * uxTaskGetSystemState() output into a human readable table that displays task
 * names, states and stack usage.
 *
 * vTaskList() has a dependency on the sprintf() C library function that might
 * bloat the code size, use a lot of stack, and provide different results on
 * different platforms.  An alternative, tiny, third party, and limited
 * functionality implementation of sprintf() is provided in many of the
 * FreeRTOS/Demo sub-directories in a file called printf-stdarg.c (note
 * printf-stdarg.c does not provide a full snprintf() implementation!).
 *
 * It is recommended that production systems call uxTaskGetSystemState()
 * directly to get access to raw stats data, rather than indirectly through a
 * call to vTaskList().
 *
 * @param pcWriteBuffer A buffer into which the above mentioned details
 * will be written, in ASCII form.  This buffer is assumed to be large
 * enough to contain the generated report.  Approximately 40 bytes per
 * task should be sufficient.
 *
 * \ingroup TaskUtils
 */
#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) )

	void vTaskList( char * pcWriteBuffer )
	{
        //暂时啥也不干
        rt_kprintf("error : vTaskList not implemented!!!\n\n");
    }
#endif /* ( ( configUSE_TRACE_FACILITY == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) ) */


/**
 * Get the state of running tasks as a string
 *
 * configGENERATE_RUN_TIME_STATS and configUSE_STATS_FORMATTING_FUNCTIONS
 * must both be defined as 1 for this function to be available.  The application
 * must also then provide definitions for
 * portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() and portGET_RUN_TIME_COUNTER_VALUE()
 * to configure a peripheral timer/counter and return the timers current count
 * value respectively.  The counter should be at least 10 times the frequency of
 * the tick count.
 *
 * @note This function will disable interrupts for its duration.  It is
 * not intended for normal application runtime use but as a debug aid.
 *
 * Setting configGENERATE_RUN_TIME_STATS to 1 will result in a total
 * accumulated execution time being stored for each task.  The resolution
 * of the accumulated time value depends on the frequency of the timer
 * configured by the portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() macro.
 * Calling vTaskGetRunTimeStats() writes the total execution time of each
 * task into a buffer, both as an absolute count value and as a percentage
 * of the total system execution time.
 *
 * @note This function is provided for convenience only, and is used by many of the
 * demo applications.  Do not consider it to be part of the scheduler.
 *
 * vTaskGetRunTimeStats() calls uxTaskGetSystemState(), then formats part of the
 * uxTaskGetSystemState() output into a human readable table that displays the
 * amount of time each task has spent in the Running state in both absolute and
 * percentage terms.
 *
 * vTaskGetRunTimeStats() has a dependency on the sprintf() C library function
 * that might bloat the code size, use a lot of stack, and provide different
 * results on different platforms.  An alternative, tiny, third party, and
 * limited functionality implementation of sprintf() is provided in many of the
 * FreeRTOS/Demo sub-directories in a file called printf-stdarg.c (note
 * printf-stdarg.c does not provide a full snprintf() implementation!).
 *
 * It is recommended that production systems call uxTaskGetSystemState() directly
 * to get access to raw stats data, rather than indirectly through a call to
 * vTaskGetRunTimeStats().
 *
 * @param pcWriteBuffer A buffer into which the execution times will be
 * written, in ASCII form.  This buffer is assumed to be large enough to
 * contain the generated report.  Approximately 40 bytes per task should
 * be sufficient.
 *
 * \ingroup TaskUtils
 */
#if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) )
	void vTaskGetRunTimeStats( char *pcWriteBuffer )
	{
        //暂时啥也不干
        rt_kprintf("error : vTaskGetRunTimeStats not implemented!!!\n\n");
    }
#endif /* ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) ) */


/**
 * Send task notification.发送任务通知。
 *
 * configUSE_TASK_NOTIFICATIONS must be undefined or defined as 1 for this
 * function to be available.
 *
 * When configUSE_TASK_NOTIFICATIONS is set to one each task has its own private
 * "notification value", which is a 32-bit unsigned integer (uint32_t).
 *
 * 可以使用中间对象将事件发送到任务。 这些对象的例子有队列、信号灯、互斥和事件组。
 * 任务通知是一种直接向任务发送事件的方法，不需要这样的中介对象。
 *
 * 发送给任务的通知可以选择性地执行一个动作，如更新、覆盖或增加任务的通知值。
 * 这样一来，任务通知可以用来向任务发送数据，也可以作为轻量级的快速二进制或计数信号符使用。
 *
 * 发送给任务的通知将一直处于等待状态，直到任务调用xTaskNotifyWait()或ulTaskNotifyTake()将其清除。
 * 如果任务在通知到达时已经处于阻塞状态等待通知，那么任务将自动从阻塞状态中移除（解除阻塞）并清除通知。
 *
 * 一个任务可以使用xTaskNotifyWait()来[可选]阻塞等待一个待定的通知，
 * 或者使用ulTaskNotifyTake()来[可选]阻塞等待其通知值具有一个非零值。 当任务处于阻塞状态时，它不会消耗任何CPU时间。
 *
 * See http://www.FreeRTOS.org/RTOS-task-notifications.html for details.
 *
 * @param xTaskToNotify 被通知的任务的句柄。 任务的句柄可以从用于创建任务的xTaskCreate()API函数中返回，
 * 当前运行的任务的句柄可以通过调用xTaskGetCurrentTaskHandle()获得。
 *
 * @param ulValue 可以与通知一起发送的数据。 如何使用这些数据取决于eAction参数的值。
 *
 * @param eAction 指定通知如何更新任务的通知值，如果有的话。 eAction的有效值如下：
 *	- eSetBits:
 *	  The task's notification value is bitwise ORed with ulValue.  xTaskNofify()
 * 	  always returns pdPASS in this case.
 *
 *	- eIncrement:
 *    任务的通知值增加，ulValue不使用，且xTaskNofify()在这种情况下总是返回pdPASS。
 *	  The task's notification value is incremented.  ulValue is not used and
 *	  xTaskNotify() always returns pdPASS in this case.
 *
 *	- eSetValueWithOverwrite:
 *    即使被通知的任务尚未处理之前的通知（该任务已经有一个待处理的通知），任务的通知值也会被设置为ulValue的值。在这种情况下，xTaskNotify()总是返回pdPASS
 *
 *	- eSetValueWithoutOverwrite:
 *	  If the task being notified did not already have a notification pending then
 *	  the task's notification value is set to ulValue and xTaskNotify() will
 *	  return pdPASS.  If the task being notified already had a notification
 *	  pending then no action is performed and pdFAIL is returned.
 *
 *	- eNoAction:
 *	  The task receives a notification without its notification value being
 *	  updated.  ulValue is not used and xTaskNotify() always returns pdPASS in
 *	  this case.
 *
 * @return 取决于eAction的值。 见eAction参数的说明。
 *
 * \ingroup TaskNotifications
 */
#if( configUSE_TASK_NOTIFICATIONS == 1 )
	BaseType_t xTaskNotify( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction )
	{
        //目前就pthread中会用到，暂时不管
        rt_kprintf("error : xTaskNotify not implemented!!!\n\n");
        return pdPASS;
    }
#endif /* configUSE_TASK_NOTIFICATIONS */


/**
 * Send task notification from an ISR.
 *
 * configUSE_TASK_NOTIFICATIONS must be undefined or defined as 1 for this
 * function to be available.
 *
 * When configUSE_TASK_NOTIFICATIONS is set to one each task has its own private
 * "notification value", which is a 32-bit unsigned integer (uint32_t).
 *
 * A version of xTaskNotify() that can be used from an interrupt service routine
 * (ISR).
 *
 * Events can be sent to a task using an intermediary object.  Examples of such
 * objects are queues, semaphores, mutexes and event groups.  Task notifications
 * are a method of sending an event directly to a task without the need for such
 * an intermediary object.
 *
 * A notification sent to a task can optionally perform an action, such as
 * update, overwrite or increment the task's notification value.  In that way
 * task notifications can be used to send data to a task, or be used as light
 * weight and fast binary or counting semaphores.
 *
 * A notification sent to a task will remain pending until it is cleared by the
 * task calling xTaskNotifyWait() or ulTaskNotifyTake().  If the task was
 * already in the Blocked state to wait for a notification when the notification
 * arrives then the task will automatically be removed from the Blocked state
 * (unblocked) and the notification cleared.
 *
 * A task can use xTaskNotifyWait() to [optionally] block to wait for a
 * notification to be pending, or ulTaskNotifyTake() to [optionally] block
 * to wait for its notification value to have a non-zero value.  The task does
 * not consume any CPU time while it is in the Blocked state.
 *
 * See http://www.FreeRTOS.org/RTOS-task-notifications.html for details.
 *
 * @param xTaskToNotify The handle of the task being notified.  The handle to a
 * task can be returned from the xTaskCreate() API function used to create the
 * task, and the handle of the currently running task can be obtained by calling
 * xTaskGetCurrentTaskHandle().
 *
 * @param ulValue Data that can be sent with the notification.  How the data is
 * used depends on the value of the eAction parameter.
 *
 * @param eAction Specifies how the notification updates the task's notification
 * value, if at all.  Valid values for eAction are as follows:
 *	- eSetBits:
 *	  The task's notification value is bitwise ORed with ulValue.  xTaskNofify()
 * 	  always returns pdPASS in this case.
 *
 *	- eIncrement:
 *	  The task's notification value is incremented.  ulValue is not used and
 *	  xTaskNotify() always returns pdPASS in this case.
 *
 *	- eSetValueWithOverwrite:
 *	  The task's notification value is set to the value of ulValue, even if the
 *	  task being notified had not yet processed the previous notification (the
 *	  task already had a notification pending).  xTaskNotify() always returns
 *	  pdPASS in this case.
 *
 *	- eSetValueWithoutOverwrite:
 *	  If the task being notified did not already have a notification pending then
 *	  the task's notification value is set to ulValue and xTaskNotify() will
 *	  return pdPASS.  If the task being notified already had a notification
 *	  pending then no action is performed and pdFAIL is returned.
 *
 *	- eNoAction:
 *	  The task receives a notification without its notification value being
 *	  updated.  ulValue is not used and xTaskNotify() always returns pdPASS in
 *	  this case.
 *
 * @param pxHigherPriorityTaskWoken  xTaskNotifyFromISR() will set
 * *pxHigherPriorityTaskWoken to pdTRUE if sending the notification caused the
 * task to which the notification was sent to leave the Blocked state, and the
 * unblocked task has a priority higher than the currently running task.  If
 * xTaskNotifyFromISR() sets this value to pdTRUE then a context switch should
 * be requested before the interrupt is exited.  How a context switch is
 * requested from an ISR is dependent on the port - see the documentation page
 * for the port in use.
 *
 * @return Dependent on the value of eAction.  See the description of the
 * eAction parameter.
 *
 * \ingroup TaskNotifications
 */
BaseType_t xTaskNotifyFromISR( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, BaseType_t *pxHigherPriorityTaskWoken )
{
    //暂时啥也不干
    rt_kprintf("error : xTaskNotifyFromISR not implemented!!!\n\n");
    return pdPASS;
}

/**
 * Wait for task notification
 *
 * configUSE_TASK_NOTIFICATIONS must be undefined or defined as 1 for this
 * function to be available.
 *
 * When configUSE_TASK_NOTIFICATIONS is set to one each task has its own private
 * "notification value", which is a 32-bit unsigned integer (uint32_t).
 *
 * Events can be sent to a task using an intermediary object.  Examples of such
 * objects are queues, semaphores, mutexes and event groups.  Task notifications
 * are a method of sending an event directly to a task without the need for such
 * an intermediary object.
 *
 * A notification sent to a task can optionally perform an action, such as
 * update, overwrite or increment the task's notification value.  In that way
 * task notifications can be used to send data to a task, or be used as light
 * weight and fast binary or counting semaphores.
 *
 * A notification sent to a task will remain pending until it is cleared by the
 * task calling xTaskNotifyWait() or ulTaskNotifyTake().  If the task was
 * already in the Blocked state to wait for a notification when the notification
 * arrives then the task will automatically be removed from the Blocked state
 * (unblocked) and the notification cleared.
 *
 * A task can use xTaskNotifyWait() to [optionally] block to wait for a
 * notification to be pending, or ulTaskNotifyTake() to [optionally] block
 * to wait for its notification value to have a non-zero value.  The task does
 * not consume any CPU time while it is in the Blocked state.
 *
 * See http://www.FreeRTOS.org/RTOS-task-notifications.html for details.
 *
 * @param ulBitsToClearOnEntry Bits that are set in ulBitsToClearOnEntry value
 * will be cleared in the calling task's notification value before the task
 * checks to see if any notifications are pending, and optionally blocks if no
 * notifications are pending.  Setting ulBitsToClearOnEntry to ULONG_MAX (if
 * limits.h is included) or 0xffffffffUL (if limits.h is not included) will have
 * the effect of resetting the task's notification value to 0.  Setting
 * ulBitsToClearOnEntry to 0 will leave the task's notification value unchanged.
 *
 * @param ulBitsToClearOnExit If a notification is pending or received before
 * the calling task exits the xTaskNotifyWait() function then the task's
 * notification value (see the xTaskNotify() API function) is passed out using
 * the pulNotificationValue parameter.  Then any bits that are set in
 * ulBitsToClearOnExit will be cleared in the task's notification value (note
 * *pulNotificationValue is set before any bits are cleared).  Setting
 * ulBitsToClearOnExit to ULONG_MAX (if limits.h is included) or 0xffffffffUL
 * (if limits.h is not included) will have the effect of resetting the task's
 * notification value to 0 before the function exits.  Setting
 * ulBitsToClearOnExit to 0 will leave the task's notification value unchanged
 * when the function exits (in which case the value passed out in
 * pulNotificationValue will match the task's notification value).
 *
 * @param pulNotificationValue Used to pass the task's notification value out
 * of the function.  Note the value passed out will not be effected by the
 * clearing of any bits caused by ulBitsToClearOnExit being non-zero.
 *
 * @param xTicksToWait The maximum amount of time that the task should wait in
 * the Blocked state for a notification to be received, should a notification
 * not already be pending when xTaskNotifyWait() was called.  The task
 * will not consume any processing time while it is in the Blocked state.  This
 * is specified in kernel ticks, the macro pdMS_TO_TICSK( value_in_ms ) can be
 * used to convert a time specified in milliseconds to a time specified in
 * ticks.
 *
 * @return If a notification was received (including notifications that were
 * already pending when xTaskNotifyWait was called) then pdPASS is
 * returned.  Otherwise pdFAIL is returned.
 *
 * \ingroup TaskNotifications
 */
#if( configUSE_TASK_NOTIFICATIONS == 1 )
	BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait )
	{
        //目前就pthread中会用到，暂时不管
        rt_kprintf("error : xTaskNotifyWait not implemented!!!\n\n");
        return pdPASS;
    }
#endif /* configUSE_TASK_NOTIFICATIONS */


/**
 * Simplified macro for sending task notification from ISR.
 *
 * configUSE_TASK_NOTIFICATIONS must be undefined or defined as 1 for this macro
 * to be available.
 *
 * When configUSE_TASK_NOTIFICATIONS is set to one each task has its own private
 * "notification value", which is a 32-bit unsigned integer (uint32_t).
 *
 * A version of xTaskNotifyGive() that can be called from an interrupt service
 * routine (ISR).
 *
 * Events can be sent to a task using an intermediary object.  Examples of such
 * objects are queues, semaphores, mutexes and event groups.  Task notifications
 * are a method of sending an event directly to a task without the need for such
 * an intermediary object.
 *
 * A notification sent to a task can optionally perform an action, such as
 * update, overwrite or increment the task's notification value.  In that way
 * task notifications can be used to send data to a task, or be used as light
 * weight and fast binary or counting semaphores.
 *
 * vTaskNotifyGiveFromISR() is intended for use when task notifications are
 * used as light weight and faster binary or counting semaphore equivalents.
 * Actual FreeRTOS semaphores are given from an ISR using the
 * xSemaphoreGiveFromISR() API function, the equivalent action that instead uses
 * a task notification is vTaskNotifyGiveFromISR().
 *
 * When task notifications are being used as a binary or counting semaphore
 * equivalent then the task being notified should wait for the notification
 * using the ulTaskNotificationTake() API function rather than the
 * xTaskNotifyWait() API function.
 *
 * See http://www.FreeRTOS.org/RTOS-task-notifications.html for more details.
 *
 * @param xTaskToNotify The handle of the task being notified.  The handle to a
 * task can be returned from the xTaskCreate() API function used to create the
 * task, and the handle of the currently running task can be obtained by calling
 * xTaskGetCurrentTaskHandle().
 *
 * @param pxHigherPriorityTaskWoken  vTaskNotifyGiveFromISR() will set
 * *pxHigherPriorityTaskWoken to pdTRUE if sending the notification caused the
 * task to which the notification was sent to leave the Blocked state, and the
 * unblocked task has a priority higher than the currently running task.  If
 * vTaskNotifyGiveFromISR() sets this value to pdTRUE then a context switch
 * should be requested before the interrupt is exited.  How a context switch is
 * requested from an ISR is dependent on the port - see the documentation page
 * for the port in use.
 *
 * \ingroup TaskNotifications
 */
#if( configUSE_TASK_NOTIFICATIONS == 1 )
	void vTaskNotifyGiveFromISR( TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken )
	{
        rt_kprintf("error : vTaskNotifyGiveFromISR not implemented!!!\n\n");
        return ;
	}
#endif /* configUSE_TASK_NOTIFICATIONS */


/**
 * Simplified macro for receiving task notification.
 *
 * configUSE_TASK_NOTIFICATIONS must be undefined or defined as 1 for this
 * function to be available.
 *
 * When configUSE_TASK_NOTIFICATIONS is set to one each task has its own private
 * "notification value", which is a 32-bit unsigned integer (uint32_t).
 *
 * Events can be sent to a task using an intermediary object.  Examples of such
 * objects are queues, semaphores, mutexes and event groups.  Task notifications
 * are a method of sending an event directly to a task without the need for such
 * an intermediary object.
 *
 * A notification sent to a task can optionally perform an action, such as
 * update, overwrite or increment the task's notification value.  In that way
 * task notifications can be used to send data to a task, or be used as light
 * weight and fast binary or counting semaphores.
 *
 * ulTaskNotifyTake() is intended for use when a task notification is used as a
 * faster and lighter weight binary or counting semaphore alternative.  Actual
 * FreeRTOS semaphores are taken using the xSemaphoreTake() API function, the
 * equivalent action that instead uses a task notification is
 * ulTaskNotifyTake().
 *
 * When a task is using its notification value as a binary or counting semaphore
 * other tasks should send notifications to it using the xTaskNotifyGive()
 * macro, or xTaskNotify() function with the eAction parameter set to
 * eIncrement.
 *
 * ulTaskNotifyTake() can either clear the task's notification value to
 * zero on exit, in which case the notification value acts like a binary
 * semaphore, or decrement the task's notification value on exit, in which case
 * the notification value acts like a counting semaphore.
 *
 * A task can use ulTaskNotifyTake() to [optionally] block to wait for a
 * the task's notification value to be non-zero.  The task does not consume any
 * CPU time while it is in the Blocked state.
 *
 * Where as xTaskNotifyWait() will return when a notification is pending,
 * ulTaskNotifyTake() will return when the task's notification value is
 * not zero.
 *
 * See http://www.FreeRTOS.org/RTOS-task-notifications.html for details.
 *
 * @param xClearCountOnExit if xClearCountOnExit is pdFALSE then the task's
 * notification value is decremented when the function exits.  In this way the
 * notification value acts like a counting semaphore.  If xClearCountOnExit is
 * not pdFALSE then the task's notification value is cleared to zero when the
 * function exits.  In this way the notification value acts like a binary
 * semaphore.
 *
 * @param xTicksToWait The maximum amount of time that the task should wait in
 * the Blocked state for the task's notification value to be greater than zero,
 * should the count not already be greater than zero when
 * ulTaskNotifyTake() was called.  The task will not consume any processing
 * time while it is in the Blocked state.  This is specified in kernel ticks,
 * the macro pdMS_TO_TICSK( value_in_ms ) can be used to convert a time
 * specified in milliseconds to a time specified in ticks.
 *
 * @return The task's notification count before it is either cleared to zero or
 * decremented (see the xClearCountOnExit parameter).
 *
 * \ingroup TaskNotifications
 */
#if( configUSE_TASK_NOTIFICATIONS == 1 )

	uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit, TickType_t xTicksToWait )
	{
        rt_kprintf("error : ulTaskNotifyTake not implemented!!!\n\n");
        return 0;
    }
#endif /* configUSE_TASK_NOTIFICATIONS */
/*-----------------------------------------------------------
 * SCHEDULER INTERNALS AVAILABLE FOR PORTING PURPOSES
 *----------------------------------------------------------*/
/** @cond */
/*
 * THIS FUNCTION MUST NOT BE USED FROM APPLICATION CODE.  IT IS ONLY
 * INTENDED FOR USE WHEN IMPLEMENTING A PORT OF THE SCHEDULER AND IS
 * AN INTERFACE WHICH IS FOR THE EXCLUSIVE USE OF THE SCHEDULER.
 *
 * Called from the real time kernel tick (either preemptive or cooperative),
 * this increments the tick count and checks if any tasks that are blocked
 * for a finite period required removing from a blocked list and placing on
 * a ready list.  If a non-zero value is returned then a context switch is
 * required because either:
 *   + A task was removed from a blocked list because its timeout had expired,
 *     or
 *   + Time slicing is in use and there is a task of equal priority to the
 *     currently running task.
 */
BaseType_t xTaskIncrementTick( void )
{
    rt_tick_increase();
    return pdFALSE;//不需要调度
}

/*
 * THIS FUNCTION MUST NOT BE USED FROM APPLICATION CODE.  IT IS AN
 * INTERFACE WHICH IS FOR THE EXCLUSIVE USE OF THE SCHEDULER.
 *
 * THIS FUNCTION MUST BE CALLED WITH INTERRUPTS DISABLED.
 *
 * Removes the calling task from the ready list and places it both
 * on the list of tasks waiting for a particular event, and the
 * list of delayed tasks.  The task will be removed from both lists
 * and replaced on the ready list should either the event occur (and
 * there be no higher priority tasks waiting on the same event) or
 * the delay period expires.
 *
 * The 'unordered' version replaces the event list item value with the
 * xItemValue value, and inserts the list item at the end of the list.
 *
 * The 'ordered' version uses the existing event list item value (which is the
 * owning tasks priority) to insert the list item into the event list is task
 * priority order.
 *
 * @param pxEventList The list containing tasks that are blocked waiting
 * for the event to occur.
 *
 * @param xItemValue The item value to use for the event list item when the
 * event list is not ordered by task priority.
 *
 * @param xTicksToWait The maximum amount of time that the task should wait
 * for the event to occur.  This is specified in kernel ticks,the constant
 * portTICK_PERIOD_MS can be used to convert kernel ticks into a real time
 * period.
 */
// void vTaskPlaceOnEventList( List_t * const pxEventList, const TickType_t xTicksToWait ) PRIVILEGED_FUNCTION;
// void vTaskPlaceOnUnorderedEventList( List_t * pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait ) PRIVILEGED_FUNCTION;

/*
 * THIS FUNCTION MUST NOT BE USED FROM APPLICATION CODE.  IT IS AN
 * INTERFACE WHICH IS FOR THE EXCLUSIVE USE OF THE SCHEDULER.
 *
 * THIS FUNCTION MUST BE CALLED WITH INTERRUPTS DISABLED.
 *
 * This function performs nearly the same function as vTaskPlaceOnEventList().
 * The difference being that this function does not permit tasks to block
 * indefinitely, whereas vTaskPlaceOnEventList() does.
 *
 */
// void vTaskPlaceOnEventListRestricted( List_t * const pxEventList, const TickType_t xTicksToWait ) PRIVILEGED_FUNCTION;

/*
 * THIS FUNCTION MUST NOT BE USED FROM APPLICATION CODE.  IT IS AN
 * INTERFACE WHICH IS FOR THE EXCLUSIVE USE OF THE SCHEDULER.
 *
 * THIS FUNCTION MUST BE CALLED WITH INTERRUPTS DISABLED.
 *
 * Removes a task from both the specified event list and the list of blocked
 * tasks, and places it on a ready queue.
 *
 * xTaskRemoveFromEventList()/xTaskRemoveFromUnorderedEventList() will be called
 * if either an event occurs to unblock a task, or the block timeout period
 * expires.
 *
 * xTaskRemoveFromEventList() is used when the event list is in task priority
 * order.  It removes the list item from the head of the event list as that will
 * have the highest priority owning task of all the tasks on the event list.
 * xTaskRemoveFromUnorderedEventList() is used when the event list is not
 * ordered and the event list items hold something other than the owning tasks
 * priority.  In this case the event list item value is updated to the value
 * passed in the xItemValue parameter.
 *
 * @return pdTRUE if the task being removed has a higher priority than the task
 * making the call, otherwise pdFALSE.
 */
// BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList ) PRIVILEGED_FUNCTION;
// BaseType_t xTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem, const TickType_t xItemValue ) PRIVILEGED_FUNCTION;

/*
 * THIS FUNCTION MUST NOT BE USED FROM APPLICATION CODE.  IT IS ONLY
 * INTENDED FOR USE WHEN IMPLEMENTING A PORT OF THE SCHEDULER AND IS
 * AN INTERFACE WHICH IS FOR THE EXCLUSIVE USE OF THE SCHEDULER.
 *
 * 将指向当前TCB的指针设置为准备运行的最高优先级任务的TCB
 */
// void vTaskSwitchContext( void ) PRIVILEGED_FUNCTION;

/*
 * THESE FUNCTIONS MUST NOT BE USED FROM APPLICATION CODE.  THEY ARE USED BY
 * THE EVENT BITS MODULE.
 */
// TickType_t uxTaskResetEventItemValue( void ) PRIVILEGED_FUNCTION;

/*
 * Return the handle of the calling task.
 */
#if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) )
	TaskHandle_t xTaskGetCurrentTaskHandle( void )
	{
        return rt_thread_self();
	}

	TaskHandle_t xTaskGetCurrentTaskHandleForCPU( BaseType_t cpuid )
	{
        return rt_thread_self();
	}
#endif /* ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */


/*
 * Return the handle of the task running on a certain CPU. Because of
 * the nature of SMP processing, there is no guarantee that this
 * value will still be valid on return and should only be used for
 * debugging purposes.
 */
// TaskHandle_t xTaskGetCurrentTaskHandleForCPU( BaseType_t cpuid );


/*
 * Capture the current time status for future reference.
 */
// void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) PRIVILEGED_FUNCTION;

/*
 * Compare the time status now with that previously captured to see if the
 * timeout has expired.
 */
// BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut, TickType_t * const pxTicksToWait ) PRIVILEGED_FUNCTION;

/*
 * Shortcut used by the queue implementation to prevent unnecessary call to
 * taskYIELD();
 */
// void vTaskMissedYield( void ) PRIVILEGED_FUNCTION;

/*
 * Returns the scheduler state as taskSCHEDULER_RUNNING,
 * taskSCHEDULER_NOT_STARTED or taskSCHEDULER_SUSPENDED.
 */
#if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
	BaseType_t xTaskGetSchedulerState( void )
	{
        BaseType_t xReturn;

        //未考虑 taskSCHEDULER_NOT_STARTED 情况，可能存在bug
        //在 os 没有起来的时候会newlib通过该函数获得系统是否已经运行起来
        if( xSchedulerRunning == pdFALSE )
		{
			xReturn = taskSCHEDULER_NOT_STARTED;
		}else if(rt_critical_level() > 0)
        {
            xReturn = taskSCHEDULER_SUSPENDED;
        }else
        {
            xReturn = taskSCHEDULER_RUNNING;
        }
        return xReturn;
	}
#endif /* ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) ) */

/*
 * Raises the priority of the mutex holder to that of the calling task should
 * the mutex holder have a priority less than the calling task.
 */
// void vTaskPriorityInherit( TaskHandle_t const pxMutexHolder ) PRIVILEGED_FUNCTION;

/*
 * Set the priority of a task back to its proper priority in the case that it
 * inherited a higher priority while it was holding a semaphore.
 */
// BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder ) PRIVILEGED_FUNCTION;

/*-----------------------------------------------------------*/

/* 对于多核来说，这是假设vPortCPUAquireMutex是递归的，也就是说，它可以被多次调用，而释放调用必须调用同样多的次数才能使mux解锁。*/
/* For multicore, this assumes the vPortCPUAquireMutex is recursive, that is, it can be called multiple
   times and the release call will have to be called as many times for the mux to unlock. */

// 疑难杂症(根据http://www.freertos.org/FreeRTOS_Support_Forum_Archive/December_2012/freertos_PIC32_Bug_-_vTaskEnterCritical_6400806.html，这似乎是FreeRTOS中故意的)是，
// 当调度器不运行时，调用vTaskEnterCritical后再调用vTaskExitCritical会使中断被禁用，重新启用调度器会重新启用中断。
/* Gotcha (which seems to be deliberate in FreeRTOS, according to
http://www.freertos.org/FreeRTOS_Support_Forum_Archive/December_2012/freertos_PIC32_Bug_-_vTaskEnterCritical_6400806.html
) is that calling vTaskEnterCritical followed by vTaskExitCritical will leave the interrupts DISABLED when the scheduler
is not running.  Re-enabling the scheduler will re-enable the interrupts instead.

对于ESP32 FreeRTOS，vTaskEnterCritical同时实现了portENTER_CRITICAL和portENTER_CRITICAL_ISR。
For ESP32 FreeRTOS, vTaskEnterCritical implements both portENTER_CRITICAL and portENTER_CRITICAL_ISR.
*/


// 对于单核来说 仅仅需要关闭中断即可
// 进入临界区后会关闭中断 此时不能调度 故这里通过一个全局变量记录中断状态标志位 用于最后嵌套返回时恢复中断
// 该函数在中断和线程上下文均可进入 同时还需满足可嵌套调用
#if ( portCRITICAL_NESTING_IN_TCB == 1 )

// #include "portmux_impl.h"
volatile rt_base_t level = 0;//中断标志位
volatile rt_base_t CriticalNesting = 0;//当前嵌套层数

// #ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
// 	void vTaskEnterCritical( portMUX_TYPE *mux, const char *function, int line )
// #else
// 	void vTaskEnterCritical( portMUX_TYPE *mux )
// #endif
// 	{
// 		BaseType_t oldInterruptLevel=0;
// 		BaseType_t schedulerRunning = xSchedulerRunning;
// 		if( schedulerRunning != pdFALSE )
// 		{
// 			//Interrupts may already be disabled (because we're doing this recursively) but we can't get the interrupt level after
// 			//vPortCPUAquireMutex, because it also may mess with interrupts. Get it here first, then later figure out if we're nesting
// 			//and save for real there.
// 			oldInterruptLevel=portENTER_CRITICAL_NESTED();
// 		}
//
// #ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
// 		vPortCPUAcquireMutexIntsDisabled( mux, portMUX_NO_TIMEOUT, function, line );
// #else
// 		vPortCPUAcquireMutexIntsDisabled( mux, portMUX_NO_TIMEOUT );
// #endif

// 		if( schedulerRunning != pdFALSE )
// 		{
// 			TCB_t *tcb = pxCurrentTCB[xPortGetCoreID()];
// 			BaseType_t newNesting = tcb->uxCriticalNesting + 1;
//  			tcb->uxCriticalNesting = newNesting;
// 			if( newNesting == 1 )
// 			{
// 				//This is the first time we get called. Save original interrupt level.
// 				tcb->uxOldInterruptState = oldInterruptLevel;
// 			}

// 			/* Original FreeRTOS comment, saved for reference:
// 			This is not the interrupt safe version of the enter critical
// 			function so	assert() if it is being called from an interrupt
// 			context.  Only API functions that end in "FromISR" can be used in an
// 			interrupt. Only assert if the critical nesting count is 1 to
// 			protect against recursive calls if the assert function also uses a
// 			critical section. */

// 			/* DISABLED in the esp32 port - because of SMP, For ESP32
// 			FreeRTOS, vTaskEnterCritical implements both
// 			portENTER_CRITICAL and portENTER_CRITICAL_ISR. vTaskEnterCritical
// 			has to be used in way more places than before, and some are called
// 			both from ISR as well as non-ISR code, thus we re-organized
// 			vTaskEnterCritical to also work in ISRs. */
// #if 0
// 			if( newNesting	== 1 )
// 			{
// 				portASSERT_IF_IN_ISR();
// 			}
// #endif

// 		}
// 		else
// 		{
// 			mtCOVERAGE_TEST_MARKER();
// 		}
// 	}

#ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
	void vTaskEnterCritical( portMUX_TYPE *mux, const char *function, int line )
#else
	void vTaskEnterCritical( portMUX_TYPE *mux )
#endif
	{
        register rt_base_t oldInterruptLevel;//中断标志位
        /* disable interrupt */
        oldInterruptLevel = rt_hw_interrupt_disable();

        CriticalNesting++;
        if(CriticalNesting == 1)
        {
            level = oldInterruptLevel;
        }
	}


#endif /* portCRITICAL_NESTING_IN_TCB */
/*-----------------------------------------------------------*/


/*
For ESP32 FreeRTOS, vTaskExitCritical implements both portEXIT_CRITICAL and portEXIT_CRITICAL_ISR.
*/
#if ( portCRITICAL_NESTING_IN_TCB == 1 )

// #ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
// 	void vTaskExitCritical( portMUX_TYPE *mux, const char *function, int line )
// #else
// 	void vTaskExitCritical( portMUX_TYPE *mux )
// #endif
// 	{
// #ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
// 		vPortCPUReleaseMutexIntsDisabled( mux, function, line );
// #else
// 		vPortCPUReleaseMutexIntsDisabled( mux );
// #endif
// 		if( xSchedulerRunning != pdFALSE )
// 		{
// 			TCB_t *tcb = pxCurrentTCB[xPortGetCoreID()];
// 			BaseType_t nesting = tcb->uxCriticalNesting;
// 			if( nesting	 > 0U )
// 			{
// 				nesting--;
// 				tcb->uxCriticalNesting = nesting;

// 				if( nesting == 0U )
// 				{
// 					portEXIT_CRITICAL_NESTED(tcb->uxOldInterruptState);
// 				}
// 				else
// 				{
// 					mtCOVERAGE_TEST_MARKER();
// 				}
// 			}
// 			else
// 			{
// 				mtCOVERAGE_TEST_MARKER();
// 			}
// 		}
// 		else
// 		{
// 			mtCOVERAGE_TEST_MARKER();
// 		}
// 	}

//恢复临界区
#ifdef CONFIG_FREERTOS_PORTMUX_DEBUG
	void vTaskExitCritical( portMUX_TYPE *mux, const char *function, int line )
#else
	void vTaskExitCritical( portMUX_TYPE *mux )
#endif
	{
        if(CriticalNesting > 0U)
        {
            CriticalNesting--;
            if(CriticalNesting == 0U)
            {
                /* enable interrupt */
                rt_hw_interrupt_enable(level);
            }
        }
	}

#endif /* portCRITICAL_NESTING_IN_TCB */
/*-----------------------------------------------------------*/




/*
 * Get the uxTCBNumber assigned to the task referenced by the xTask parameter.
 */
// UBaseType_t uxTaskGetTaskNumber( TaskHandle_t xTask ) PRIVILEGED_FUNCTION;


/*
 * Get the current core affinity of a task
 */
BaseType_t xTaskGetAffinity( TaskHandle_t xTask )
{
    rt_kprintf("error : xTaskGetAffinity not implemented!!!\n\n");
    return 0;
}

/*
 * Set the uxTaskNumber of the task referenced by the xTask parameter to
 * uxHandle.
 */
// void vTaskSetTaskNumber( TaskHandle_t xTask, const UBaseType_t uxHandle ) PRIVILEGED_FUNCTION;

/*
 * Only available when configUSE_TICKLESS_IDLE is set to 1.
 * If tickless mode is being used, or a low power mode is implemented, then
 * the tick interrupt will not execute during idle periods.  When this is the
 * case, the tick count value maintained by the scheduler needs to be kept up
 * to date with the actual execution time by being skipped forward by a time
 * equal to the idle period.
 */
#if ( configUSE_TICKLESS_IDLE != 0 )
	void vTaskStepTick( const TickType_t xTicksToJump )
	{
        rt_kprintf("error : vTaskStepTick not implemented!!!\n\n");
		//暂时啥也不干
	}
#endif /* configUSE_TICKLESS_IDLE */


/*
 * Only avilable when configUSE_TICKLESS_IDLE is set to 1.
 * Provided for use within portSUPPRESS_TICKS_AND_SLEEP() to allow the port
 * specific sleep function to determine if it is ok to proceed with the sleep,
 * and if it is ok to proceed, if it is ok to sleep indefinitely.
 *
 * This function is necessary because portSUPPRESS_TICKS_AND_SLEEP() is only
 * called with the scheduler suspended, not from within a critical section.  It
 * is therefore possible for an interrupt to request a context switch between
 * portSUPPRESS_TICKS_AND_SLEEP() and the low power mode actually being
 * entered.  eTaskConfirmSleepModeStatus() should be called from a short
 * critical section between the timer being stopped and the sleep mode being
 * entered to ensure it is ok to proceed into the sleep mode.
 */
// eSleepModeStatus eTaskConfirmSleepModeStatus( void ) PRIVILEGED_FUNCTION;

/*
 * For internal use only.  Increment the mutex held count when a mutex is
 * taken and return the handle of the task that has taken the mutex.
 */
// void *pvTaskIncrementMutexHeldCount( void );

/*
 * 这个函数为系统中的每一个任务填充了TaskSnapshot_t结构的数组。
 * 由核心转储设施使用，获取系统中所有任务的快照。
 * @param pxTaskSnapshotArray 用于存储任务快照数据的 TaskSnapshot_t 结构数组的指针。
 * @param uxArraySize 任务快照数组的大小。
 * @param pxTcbSz 用于存储 TCB 大小的指针
 * @return 数组中存储的元素数量.
 *
 *
 * This function fills array with TaskSnapshot_t structures for every task in the system.
 * Used by core dump facility to get snapshots of all tasks in the system.
 * Only available when configENABLE_TASK_SNAPSHOT is set to 1.
 * @param pxTaskSnapshotArray Pointer to array of TaskSnapshot_t structures to store tasks snapshot data.
 * @param uxArraySize Size of tasks snapshots array.
 * @param pxTcbSz Pointer to store size of TCB.
 * @return Number of elements stored in array.
 */
#if ( configENABLE_TASK_SNAPSHOT == 1 )
    UBaseType_t uxTaskGetSnapshotAll( TaskSnapshot_t * const pxTaskSnapshotArray, const UBaseType_t uxArraySize, UBaseType_t * const pxTcbSz )
	{
        rt_kprintf("error : uxTaskGetSnapshotAll not implemented!!!\n\n");
        return 0;
    }
#endif

