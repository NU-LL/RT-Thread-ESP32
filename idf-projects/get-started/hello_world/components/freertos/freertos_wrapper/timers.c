#include <rthw.h>
#include <rtthread.h>


#include <stdlib.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"

//freertos中对rtt定时器的封装
struct freertos_timer
{
    struct rt_timer *rtt_timer;
    void * pvTimerID;
};
typedef struct freertos_timer *freertos_timer_t;


/**
 * Creates a new software timer instance, and returns a handle by which the
 * created software timer can be referenced.
 *
 * Internally, within the FreeRTOS implementation, software timers use a block
 * of memory, in which the timer data structure is stored.  If a software timer
 * is created using xTimerCreate() then the required memory is automatically
 * dynamically allocated inside the xTimerCreate() function.  (see
 * http://www.freertos.org/a00111.html).  If a software timer is created using
 * xTimerCreateStatic() then the application writer must provide the memory that
 * will get used by the software timer.  xTimerCreateStatic() therefore allows a
 * software timer to be created without using any dynamic memory allocation.
 *
 * Timers are created in the dormant state.  The xTimerStart(), xTimerReset(),
 * xTimerStartFromISR(), xTimerResetFromISR(), xTimerChangePeriod() and
 * xTimerChangePeriodFromISR() API functions can all be used to transition a
 * timer into the active state.
 *
 * @param pcTimerName A text name that is assigned to the timer.  This is done
 * purely to assist debugging.  The kernel itself only ever references a timer
 * by its handle, and never by its name.
 *
 * @param xTimerPeriodInTicks 定时器的周期。 时间是以滴答周期来定义的，
 * 因此常量 portTICK_PERIOD_MS 可以用来转换以毫秒为单位的时间。 例如，
 * 如果定时器必须在 100 个滴答之后过期， 则 xTimerPeriodInTicks 应设置为 100。另外，
 * 如果定时器必须在 500ms 之后过期，那么 xPeriod 可以设置为 ( 500 / portTICK_PERIOD_MS ) ，
 * 条件是 configTICK_RATE_HZ 小于或等于 1000
 *
 * @param uxAutoReload 如果uxAutoReload设置为pdTRUE，那么定时器将以xTimerPeriodInTicks参数设置的频率反复过期。
 * 如果将uxAutoReload设置为pdFALSE，那么定时器将是一次性定时器，过期后进入休眠状态。
 *
 * @param pvTimerID 分配给正在创建的定时器的标识符。通常，当同一个回调函数分配给多个定时器时，这将用于定时器回调函数，以确定哪个定时器过期
 *
 * @param pxCallbackFunction The function to call when the timer expires.
 * Callback functions must have the prototype defined by TimerCallbackFunction_t,
 * which is	"void vCallbackFunction( TimerHandle_t xTimer );".
 *
 * @return If the timer is successfully created then a handle to the newly
 * created timer is returned.  If the timer cannot be created (because either
 * there is insufficient FreeRTOS heap remaining to allocate the timer
 * structures, or the timer period was set to 0) then NULL is returned.
 *
 * Example usage:
 * @code{c}
 * #define NUM_TIMERS 5
 *
 * // An array to hold handles to the created timers.
 * TimerHandle_t xTimers[ NUM_TIMERS ];
 *
 * // An array to hold a count of the number of times each timer expires.
 * int32_t lExpireCounters[ NUM_TIMERS ] = { 0 };
 *
 * // Define a callback function that will be used by multiple timer instances.
 * // The callback function does nothing but count the number of times the
 * // associated timer expires, and stop the timer once the timer has expired
 * // 10 times.
 * void vTimerCallback( TimerHandle_t pxTimer )
 * {
 * int32_t lArrayIndex;
 * const int32_t xMaxExpiryCountBeforeStopping = 10;
 *
 * 	   // Optionally do something if the pxTimer parameter is NULL.
 * 	   configASSERT( pxTimer );
 *
 *     // Which timer expired?
 *     lArrayIndex = ( int32_t ) pvTimerGetTimerID( pxTimer );
 *
 *     // Increment the number of times that pxTimer has expired.
 *     lExpireCounters[ lArrayIndex ] += 1;
 *
 *     // If the timer has expired 10 times then stop it from running.
 *     if( lExpireCounters[ lArrayIndex ] == xMaxExpiryCountBeforeStopping )
 *     {
 *         // Do not use a block time if calling a timer API function from a
 *         // timer callback function, as doing so could cause a deadlock!
 *         xTimerStop( pxTimer, 0 );
 *     }
 * }
 *
 * void main( void )
 * {
 * int32_t x;
 *
 *     // Create then start some timers.  Starting the timers before the scheduler
 *     // has been started means the timers will start running immediately that
 *     // the scheduler starts.
 *     for( x = 0; x < NUM_TIMERS; x++ )
 *     {
 *         xTimers[ x ] = xTimerCreate(    "Timer",       // Just a text name, not used by the kernel.
 *                                         ( 100 * x ),   // The timer period in ticks.
 *                                         pdTRUE,        // The timers will auto-reload themselves when they expire.
 *                                         ( void * ) x,  // Assign each timer a unique id equal to its array index.
 *                                         vTimerCallback // Each timer calls the same callback when it expires.
 *                                     );
 *
 *         if( xTimers[ x ] == NULL )
 *         {
 *             // The timer was not created.
 *         }
 *         else
 *         {
 *             // Start the timer.  No block time is specified, and even if one was
 *             // it would be ignored because the scheduler has not yet been
 *             // started.
 *             if( xTimerStart( xTimers[ x ], 0 ) != pdPASS )
 *             {
 *                 // The timer could not be set into the Active state.
 *             }
 *         }
 *     }
 *
 *     // ...
 *     // Create tasks here.
 *     // ...
 *
 *     // Starting the scheduler will start the timers running as they have already
 *     // been set into the active state.
 *     vTaskStartScheduler();
 *
 *     // Should not reach here.
 *     for( ;; );
 * }
 * @endcode
 */
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	TimerHandle_t xTimerCreate(	const char * const pcTimerName,
								const TickType_t xTimerPeriodInTicks,
								const UBaseType_t uxAutoReload,
								void * const pvTimerID,
								TimerCallbackFunction_t pxCallbackFunction )
    {
        int i;
        freertos_timer_t timer = (freertos_timer_t)RT_KERNEL_MALLOC(sizeof(struct freertos_timer));;

        // timer.rtt_timer = rt_timer_create(pcTimerName, pxCallbackFunction, );

        /* allocate a object */
        timer->rtt_timer = (struct rt_timer *)rt_object_allocate(RT_Object_Class_Timer, pcTimerName);
        if (timer->rtt_timer == RT_NULL)
        {
            return RT_NULL;
        }

        /* set flag */
        if(pdTRUE == uxAutoReload)//反复定时
            timer->rtt_timer->parent.flag  = RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER;
        else
            timer->rtt_timer->parent.flag  = RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER;

        /* set deactivated */
        timer->rtt_timer->parent.flag &= ~RT_TIMER_FLAG_ACTIVATED;

        timer->rtt_timer->timeout_func = (void (*)(void *parameter))pxCallbackFunction;
        timer->rtt_timer->parameter    = timer.rtt_timer;

        timer->rtt_timer->timeout_tick = 0;
        timer->rtt_timer->init_tick    = xTimerPeriodInTicks;

        timer->pvTimerID = pvTimerID;

        /* initialize timer list */
        for (i = 0; i < RT_TIMER_SKIP_LIST_LEVEL; i++)
        {
            rt_list_init(&(timer->rtt_timer->row[i]));
        }

        return (void *)timer;
    }
#endif

 /**
  * Creates a new software timer instance, and returns a handle by which the
  * created software timer can be referenced.
  *
  * Internally, within the FreeRTOS implementation, software timers use a block
  * of memory, in which the timer data structure is stored.  If a software timer
  * is created using xTimerCreate() then the required memory is automatically
  * dynamically allocated inside the xTimerCreate() function.  (see
  * http://www.freertos.org/a00111.html).  If a software timer is created using
  * xTimerCreateStatic() then the application writer must provide the memory that
  * will get used by the software timer.  xTimerCreateStatic() therefore allows a
  * software timer to be created without using any dynamic memory allocation.
  *
  * Timers are created in the dormant state.  The xTimerStart(), xTimerReset(),
  * xTimerStartFromISR(), xTimerResetFromISR(), xTimerChangePeriod() and
  * xTimerChangePeriodFromISR() API functions can all be used to transition a
  * timer into the active state.
  *
  * @param pcTimerName A text name that is assigned to the timer.  This is done
  * purely to assist debugging.  The kernel itself only ever references a timer
  * by its handle, and never by its name.
  *
  * @param xTimerPeriodInTicks The timer period.  The time is defined in tick
  * periods so the constant portTICK_PERIOD_MS can be used to convert a time that
  * has been specified in milliseconds.  For example, if the timer must expire
  * after 100 ticks, then xTimerPeriodInTicks should be set to 100.
  * Alternatively, if the timer must expire after 500ms, then xPeriod can be set
  * to ( 500 / portTICK_PERIOD_MS ) provided configTICK_RATE_HZ is less than or
  * equal to 1000.
  *
  * @param uxAutoReload If uxAutoReload is set to pdTRUE then the timer will
  * expire repeatedly with a frequency set by the xTimerPeriodInTicks parameter.
  * If uxAutoReload is set to pdFALSE then the timer will be a one-shot timer and
  * enter the dormant state after it expires.
  *
  * @param pvTimerID An identifier that is assigned to the timer being created.
  * Typically this would be used in the timer callback function to identify which
  * timer expired when the same callback function is assigned to more than one
  * timer.
  *
  * @param pxCallbackFunction The function to call when the timer expires.
  * Callback functions must have the prototype defined by TimerCallbackFunction_t,
  * which is "void vCallbackFunction( TimerHandle_t xTimer );".
  *
  * @param pxTimerBuffer Must point to a variable of type StaticTimer_t, which
  * will be then be used to hold the software timer's data structures, removing
  * the need for the memory to be allocated dynamically.
  *
  * @return If the timer is created then a handle to the created timer is
  * returned.  If pxTimerBuffer was NULL then NULL is returned.
  *
  * Example usage:
  * @code{c}
  *
  * // The buffer used to hold the software timer's data structure.
  * static StaticTimer_t xTimerBuffer;
  *
  * // A variable that will be incremented by the software timer's callback
  * // function.
  * UBaseType_t uxVariableToIncrement = 0;
  *
  * // A software timer callback function that increments a variable passed to
  * // it when the software timer was created.  After the 5th increment the
  * // callback function stops the software timer.
  * static void prvTimerCallback( TimerHandle_t xExpiredTimer )
  * {
  * UBaseType_t *puxVariableToIncrement;
  * BaseType_t xReturned;
  *
  *     // Obtain the address of the variable to increment from the timer ID.
  *     puxVariableToIncrement = ( UBaseType_t * ) pvTimerGetTimerID( xExpiredTimer );
  *
  *     // Increment the variable to show the timer callback has executed.
  *     ( *puxVariableToIncrement )++;
  *
  *     // If this callback has executed the required number of times, stop the
  *     // timer.
  *     if( *puxVariableToIncrement == 5 )
  *     {
  *         // This is called from a timer callback so must not block.
  *         xTimerStop( xExpiredTimer, staticDONT_BLOCK );
  *     }
  * }
  *
  *
  * void main( void )
  * {
  *     // Create the software time.  xTimerCreateStatic() has an extra parameter
  *     // than the normal xTimerCreate() API function.  The parameter is a pointer
  *     // to the StaticTimer_t structure that will hold the software timer
  *     // structure.  If the parameter is passed as NULL then the structure will be
  *     // allocated dynamically, just as if xTimerCreate() had been called.
  *     xTimer = xTimerCreateStatic( "T1",             // Text name for the task.  Helps debugging only.  Not used by FreeRTOS.
  *                                  xTimerPeriod,     // The period of the timer in ticks.
  *                                  pdTRUE,           // This is an auto-reload timer.
  *                                  ( void * ) &uxVariableToIncrement,    // A variable incremented by the software timer's callback function
  *                                  prvTimerCallback, // The function to execute when the timer expires.
  *                                  &xTimerBuffer );  // The buffer that will hold the software timer structure.
  *
  *     // The scheduler has not started yet so a block time is not used.
  *     xReturned = xTimerStart( xTimer, 0 );
  *
  *     // ...
  *     // Create tasks here.
  *     // ...
  *
  *     // Starting the scheduler will start the timers running as they have already
  *     // been set into the active state.
  *     vTaskStartScheduler();
  *
  *     // Should not reach here.
  *     for( ;; );
  * }
  * @endcode
  */
 #if( configSUPPORT_STATIC_ALLOCATION == 1 )
	TimerHandle_t xTimerCreateStatic(	const char * const pcTimerName,
										const TickType_t xTimerPeriodInTicks,
										const UBaseType_t uxAutoReload,
										void * const pvTimerID,
										TimerCallbackFunction_t pxCallbackFunction,
										StaticTimer_t *pxTimerBuffer )
    {
        //暂时不实现
    }
 #endif /* configSUPPORT_STATIC_ALLOCATION */

/**
 * Returns the ID assigned to the timer.
 *
 * IDs are assigned to timers using the pvTimerID parameter of the call to
 * xTimerCreated() that was used to create the timer.
 *
 * If the same callback function is assigned to multiple timers then the timer
 * ID can be used within the callback function to identify which timer actually
 * expired.
 *
 * @param xTimer The timer being queried.
 *
 * @return The ID assigned to the timer being queried.
 *
 * Example usage:
 *
 * See the xTimerCreate() API function example usage scenario.
 */
void *pvTimerGetTimerID( const TimerHandle_t xTimer )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
	return pxTimer->pvTimerID;
}

/**
 * Sets the ID assigned to the timer.
 *
 * IDs are assigned to timers using the pvTimerID parameter of the call to
 * xTimerCreated() that was used to create the timer.
 *
 * If the same callback function is assigned to multiple timers then the timer
 * ID can be used as time specific (timer local) storage.
 *
 * @param xTimer The timer being updated.
 *
 * @param pvNewID The ID to assign to the timer.
 *
 * Example usage:
 *
 * See the xTimerCreate() API function example usage scenario.
 */
void vTimerSetTimerID( TimerHandle_t xTimer, void *pvNewID )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );
    //taskENTER_CRITICAL();     //Atomic instruction, critical not necessary
    //{
        pxTimer->pvTimerID = pvNewID;
    //}
    //taskEXIT_CRITICAL();
}

/**
 * Queries a timer to see if it is active or dormant.
 *
 * A timer will be dormant if:
 *
 *     1) It has been created but not started, or
 *
 *     2) It is an expired one-shot timer that has not been restarted.
 *
 * Timers are created in the dormant state.  The xTimerStart(), xTimerReset(),
 * xTimerStartFromISR(), xTimerResetFromISR(), xTimerChangePeriod() and
 * xTimerChangePeriodFromISR() API functions can all be used to transition a timer into the
 * active state.
 *
 * @param xTimer The timer being queried.
 *
 * @return pdFALSE will be returned if the timer is dormant.  A value other than
 * pdFALSE will be returned if the timer is active.
 *
 * Example usage:
 * @code{c}
 * // This function assumes xTimer has already been created.
 * void vAFunction( TimerHandle_t xTimer )
 * {
 *     if( xTimerIsTimerActive( xTimer ) != pdFALSE ) // or more simply and equivalently "if( xTimerIsTimerActive( xTimer ) )"
 *     {
 *         // xTimer is active, do something.
 *     }
 *     else
 *     {
 *         // xTimer is not active, do something else.
 *     }
 * }
 * @endcode
 */
BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );

    if (!(pxTimer->rtt_timer->parent.flag & RT_TIMER_FLAG_ACTIVATED))//未启动
        return pdFALSE;
    else
        return pdTRUE;
}

/**
 * xTimerGetTimerDaemonTaskHandle() is only available if
 * INCLUDE_xTimerGetTimerDaemonTaskHandle is set to 1 in FreeRTOSConfig.h.
 *
 * Simply returns the handle of the timer service/daemon task.  It it not valid
 * to call xTimerGetTimerDaemonTaskHandle() before the scheduler has been started.
 */
#if ( INCLUDE_xTimerGetTimerDaemonTaskHandle == 1 )
	TaskHandle_t xTimerGetTimerDaemonTaskHandle( void )
	{
		//暂时不实现
	}
#endif

/**
 * 返回计时器的周期
 *
 * @param xTimer The handle of the timer being queried.
 *
 * @return The period of the timer in ticks.
 */
TickType_t xTimerGetPeriod( TimerHandle_t xTimer )
{
    TickType_t period;
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );

    rt_timer_control(pxTimer->rtt_timer, RT_TIMER_CTRL_GET_TIME, (void *) &period);

    return period;
}

/**
 * 返回定时器过期的时间，以ticks为单位。 如果这个时间小于当前的tick数，那么到期时间就会从当前时间溢出。
 * Returns the time in ticks at which the timer will expire.  If this is less
 * than the current tick count then the expiry time has overflowed from the
 * current time.
 *
 * @param xTimer The handle of the timer being queried.
 *
 * @return 如果定时器正在运行，则返回定时器下一次过期的时间（以ticks为单位）(定时器溢出的时间)。 如果定时器没有运行，则返回值未定义。
 * If the timer is running then the time in ticks at which the timer
 * will next expire is returned.  If the timer is not running then the return
 * value is undefined.
 */
TickType_t xTimerGetExpiryTime( TimerHandle_t xTimer )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );

    return pxTimer->rtt_timer->timeout_tick;
}

/**
 * 定时器功能由定时器服务/守护任务提供。 许多公共的FreeRTOS定时器API函数通过一个叫做定时器命令队列的队列向定时器服务任务发送命令。
 * 定时器命令队列是内核本身的私有部分，应用程序代码不能直接访问。 定时器命令队列的长度由configTIMER_QUEUE_LENGTH配置常量设置。
 *
 * xTimerStart()启动一个先前使用xTimerCreate()API函数创建的定时器。
 * 如果定时器已经被启动并且已经处于活动状态，那么xTimerStart()的功能与xTimerReset()API函数相当。
 *
 * 启动定时器确保定时器处于活动状态。 如果定时器没有被停止、删除或重置，与定时器相关联的回调函数将在xTimerStart()后'n'个tick被调用，其中'n'是定时器定义的周期。
 *
 * 在调度器被启动之前调用xTimerStart()是有效的，但是当这样做的时候，定时器实际上不会启动，
 * 直到调度器被启动，而且定时器的到期时间是相对于调度器被启动的时间，而不是相对于xTimerStart()被调用的时间。
 *
 * The configUSE_TIMERS configuration constant must be set to 1 for xTimerStart()
 * to be available.
 *
 * @param xTimer The handle of the timer being started/restarted.
 *
 * @param xTicksToWait 指定在调用 xTimerStart()时，如果定时器命令队列已经满了，
 * 那么调用任务应该在阻塞状态下等待启动命令成功发送到定时器命令队列的时间，以ticks为单位。
 * 如果 xTimerStart()在调度器启动前被调用，xTicksToWait被忽略。
 * Specifies the time, in ticks, that the calling task should
 * be held in the Blocked state to wait for the start command to be successfully
 * sent to the timer command queue, should the queue already be full when
 * xTimerStart() was called.  xTicksToWait is ignored if xTimerStart() is called
 * before the scheduler is started.
 *
 * @return pdFAIL将被返回，如果启动命令不能被发送到定时器命令队列，即使在xTicksToWait滴答声过后也是如此。
 * pdPASS将被返回，如果命令成功发送到定时器命令队列。命令何时被实际处理取决于定时器服务/守护进程任务相对于系统中其他任务的优先级，
 * 尽管定时器到期时间是相对于xTimerStart()被实际调用的时间。 定时器服务/守护进程任务的优先级由configTIMER_TASK_PRIORITY配置常量设置。
 * pdFAIL will be returned if the start command could not be sent to
 * the timer command queue even after xTicksToWait ticks had passed.  pdPASS will
 * be returned if the command was successfully sent to the timer command queue.
 * When the command is actually processed will depend on the priority of the
 * timer service/daemon task relative to other tasks in the system, although the
 * timers expiry time is relative to when xTimerStart() is actually called.  The
 * timer service/daemon task priority is set by the configTIMER_TASK_PRIORITY
 * configuration constant.
 *
 * Example usage:
 *
 * See the xTimerCreate() API function example usage scenario.
 *
 */
// #define xTimerStart( xTimer, xTicksToWait ) xTimerGenericCommand( ( xTimer ), tmrCOMMAND_START, ( xTaskGetTickCount() ), NULL, ( xTicksToWait ) )

BaseType_t xTimerGenericCommand( TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t * const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );

    switch (xCommandID) {
        case tmrCOMMAND_START :
        case tmrCOMMAND_START_FROM_ISR :
        case tmrCOMMAND_RESET :
        case tmrCOMMAND_RESET_FROM_ISR :
        case tmrCOMMAND_START_DONT_TRACE :
            if(RT_EOK == rt_timer_start(pxTimer->rtt_timer))
                return pdPASS;
            break;
        case tmrCOMMAND_STOP :
		case tmrCOMMAND_STOP_FROM_ISR :
            if(RT_EOK == rt_timer_stop(pxTimer->rtt_timer))
                return pdPASS;
            break;
        //改变周期
        case tmrCOMMAND_CHANGE_PERIOD :
		case tmrCOMMAND_CHANGE_PERIOD_FROM_ISR :
            rt_timer_control(pxTimer->rtt_timer, RT_TIMER_CTRL_SET_TIME, (void *)xOptionalValue);
            break;
        case tmrCOMMAND_DELETE :
            rt_timer_delete(pxTimer->rtt_timer);//用于动态创建的定时器删除 静态暂时未支持
            break;
        default	:
            break;
    }
    return pdFAIL;
}

/**
 * Used from application interrupt service routines to defer the execution of a
 * function to the RTOS daemon task (the timer service task, hence this function
 * is implemented in timers.c and is prefixed with 'Timer').
 *
 * Ideally an interrupt service routine (ISR) is kept as short as possible, but
 * sometimes an ISR either has a lot of processing to do, or needs to perform
 * processing that is not deterministic.  In these cases
 * xTimerPendFunctionCallFromISR() can be used to defer processing of a function
 * to the RTOS daemon task.
 *
 * A mechanism is provided that allows the interrupt to return directly to the
 * task that will subsequently execute the pended callback function.  This
 * allows the callback function to execute contiguously in time with the
 * interrupt - just as if the callback had executed in the interrupt itself.
 *
 * @param xFunctionToPend The function to execute from the timer service/
 * daemon task.  The function must conform to the PendedFunction_t
 * prototype.
 *
 * @param pvParameter1 The value of the callback function's first parameter.
 * The parameter has a void * type to allow it to be used to pass any type.
 * For example, unsigned longs can be cast to a void *, or the void * can be
 * used to point to a structure.
 *
 * @param ulParameter2 The value of the callback function's second parameter.
 *
 * @param pxHigherPriorityTaskWoken As mentioned above, calling this function
 * will result in a message being sent to the timer daemon task.  If the
 * priority of the timer daemon task (which is set using
 * configTIMER_TASK_PRIORITY in FreeRTOSConfig.h) is higher than the priority of
 * the currently running task (the task the interrupt interrupted) then
 * *pxHigherPriorityTaskWoken will be set to pdTRUE within
 * xTimerPendFunctionCallFromISR(), indicating that a context switch should be
 * requested before the interrupt exits.  For that reason
 * *pxHigherPriorityTaskWoken must be initialised to pdFALSE.  See the
 * example code below.
 *
 * @return pdPASS is returned if the message was successfully sent to the
 * timer daemon task, otherwise pdFALSE is returned.
 *
 * Example usage:
 * @code{c}
 *
 *	// The callback function that will execute in the context of the daemon task.
 *  // Note callback functions must all use this same prototype.
 *  void vProcessInterface( void *pvParameter1, uint32_t ulParameter2 )
 *	{
 *		BaseType_t xInterfaceToService;
 *
 *		// The interface that requires servicing is passed in the second
 *      // parameter.  The first parameter is not used in this case.
 *		xInterfaceToService = ( BaseType_t ) ulParameter2;
 *
 *		// ...Perform the processing here...
 *	}
 *
 *	// An ISR that receives data packets from multiple interfaces
 *  void vAnISR( void )
 *	{
 *		BaseType_t xInterfaceToService, xHigherPriorityTaskWoken;
 *
 *		// Query the hardware to determine which interface needs processing.
 *		xInterfaceToService = prvCheckInterfaces();
 *
 *      // The actual processing is to be deferred to a task.  Request the
 *      // vProcessInterface() callback function is executed, passing in the
 *		// number of the interface that needs processing.  The interface to
 *		// service is passed in the second parameter.  The first parameter is
 *		// not used in this case.
 *		xHigherPriorityTaskWoken = pdFALSE;
 *		xTimerPendFunctionCallFromISR( vProcessInterface, NULL, ( uint32_t ) xInterfaceToService, &xHigherPriorityTaskWoken );
 *
 *		// If xHigherPriorityTaskWoken is now set to pdTRUE then a context
 *		// switch should be requested.  The macro used is port specific and will
 *		// be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
 *		// the documentation page for the port being used.
 *		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
 *
 *	}
 * @endcode
 */
#if( INCLUDE_xTimerPendFunctionCall == 1 )
	BaseType_t xTimerPendFunctionCallFromISR( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, BaseType_t *pxHigherPriorityTaskWoken )
	{
        //宏 xEventGroupSetBitsFromISR xEventGroupClearBitsFromISR 的实现
        //暂时不实现 事件组中调用 （仅在 freemodbus中使用）
        if(pxHigherPriorityTaskWoken != NULL)
        {
            //xEventGroupSetBitsFromISR
            *pxHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBits((EventGroupHandle_t)pvParameter1, (EventBits_t)ulParameter2);
        }
        else
        {
            //xEventGroupClearBitsFromISR
            xEventGroupClearBits((EventGroupHandle_t)pvParameter1, (EventBits_t)ulParameter2);
        }

        return pdPASS;
	}
#endif /* INCLUDE_xTimerPendFunctionCall */


 /**
  * Used to defer the execution of a function to the RTOS daemon task (the timer
  * service task, hence this function is implemented in timers.c and is prefixed
  * with 'Timer').
  *
  * @param xFunctionToPend The function to execute from the timer service/
  * daemon task.  The function must conform to the PendedFunction_t
  * prototype.
  *
  * @param pvParameter1 The value of the callback function's first parameter.
  * The parameter has a void * type to allow it to be used to pass any type.
  * For example, unsigned longs can be cast to a void *, or the void * can be
  * used to point to a structure.
  *
  * @param ulParameter2 The value of the callback function's second parameter.
  *
  * @param xTicksToWait Calling this function will result in a message being
  * sent to the timer daemon task on a queue.  xTicksToWait is the amount of
  * time the calling task should remain in the Blocked state (so not using any
  * processing time) for space to become available on the timer queue if the
  * queue is found to be full.
  *
  * @return pdPASS is returned if the message was successfully sent to the
  * timer daemon task, otherwise pdFALSE is returned.
  *
  */
#if( INCLUDE_xTimerPendFunctionCall == 1 )
	BaseType_t xTimerPendFunctionCall( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, TickType_t xTicksToWait )
	{
        //暂时不实现
	}
#endif /* INCLUDE_xTimerPendFunctionCall */

/**
 * Returns the name that was assigned to a timer when the timer was created.
 *
 * @param xTimer The handle of the timer being queried.
 *
 * @return The name assigned to the timer specified by the xTimer parameter.
 */
const char * pcTimerGetTimerName( TimerHandle_t xTimer )
{
    freertos_timer_t pxTimer = (freertos_timer_t) xTimer;
    configASSERT( xTimer );

	return pxTimer->rtt_timer->parent.name;
}

