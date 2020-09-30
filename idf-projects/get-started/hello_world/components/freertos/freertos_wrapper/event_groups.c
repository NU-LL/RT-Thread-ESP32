#include <rthw.h>
#include <rtthread.h>


/* Standard includes. */
#include <stdlib.h>

/* FreeRTOS includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"


/**
 * Create a new event group.
 *
 * Internally, within the FreeRTOS implementation, event groups use a [small]
 * block of memory, in which the event group's structure is stored.  If an event
 * groups is created using xEventGroupCreate() then the required memory is
 * automatically dynamically allocated inside the xEventGroupCreate() function.
 * (see http://www.freertos.org/a00111.html).  If an event group is created
 * using xEventGropuCreateStatic() then the application writer must instead
 * provide the memory that will get used by the event group.
 * xEventGroupCreateStatic() therefore allows an event group to be created
 * without using any dynamic memory allocation.
 *
 * Although event groups are not related to ticks, for internal implementation
 * reasons the number of bits available for use in an event group is dependent
 * on the configUSE_16_BIT_TICKS setting in FreeRTOSConfig.h.  If
 * configUSE_16_BIT_TICKS is 1 then each event group contains 8 usable bits (bit
 * 0 to bit 7).  If configUSE_16_BIT_TICKS is set to 0 then each event group has
 * 24 usable bits (bit 0 to bit 23).  The EventBits_t type is used to store
 * event bits within an event group.
 *
 * @return If the event group was created then a handle to the event group is
 * returned.  If there was insufficient FreeRTOS heap available to create the
 * event group then NULL is returned.  See http://www.freertos.org/a00111.html
 *
 * Example usage:
 * @code{c}
 * 	// Declare a variable to hold the created event group.
 * 	EventGroupHandle_t xCreatedEventGroup;
 *
 * 	// Attempt to create the event group.
 * 	xCreatedEventGroup = xEventGroupCreate();
 *
 * 	// Was the event group created successfully?
 * 	if( xCreatedEventGroup == NULL )
 * 	{
 * 		// The event group was not created because there was insufficient
 * 		// FreeRTOS heap available.
 * 	}
 * 	else
 * 	{
 * 		// The event group was created.
 * 	}
 * @endcode
 * \ingroup EventGroup
 */
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	EventGroupHandle_t xEventGroupCreate( void )
	{
        static rt_uint16_t idx = 0;
        char namebuf[16] = {0};
        sprintf( namebuf, "event%02d", idx);
        idx++;
        return rt_event_create(namebuf, RT_IPC_FLAG_FIFO);
	}
#endif /* configSUPPORT_DYNAMIC_ALLOCATION */

/**
 * Create a new event group.
 *
 * Internally, within the FreeRTOS implementation, event groups use a [small]
 * block of memory, in which the event group's structure is stored.  If an event
 * groups is created using xEventGropuCreate() then the required memory is
 * automatically dynamically allocated inside the xEventGroupCreate() function.
 * (see http://www.freertos.org/a00111.html).  If an event group is created
 * using xEventGropuCreateStatic() then the application writer must instead
 * provide the memory that will get used by the event group.
 * xEventGroupCreateStatic() therefore allows an event group to be created
 * without using any dynamic memory allocation.
 *
 * Although event groups are not related to ticks, for internal implementation
 * reasons the number of bits available for use in an event group is dependent
 * on the configUSE_16_BIT_TICKS setting in FreeRTOSConfig.h.  If
 * configUSE_16_BIT_TICKS is 1 then each event group contains 8 usable bits (bit
 * 0 to bit 7).  If configUSE_16_BIT_TICKS is set to 0 then each event group has
 * 24 usable bits (bit 0 to bit 23).  The EventBits_t type is used to store
 * event bits within an event group.
 *
 * @param pxEventGroupBuffer pxEventGroupBuffer must point to a variable of type
 * StaticEventGroup_t, which will be then be used to hold the event group's data
 * structures, removing the need for the memory to be allocated dynamically.
 *
 * @return If the event group was created then a handle to the event group is
 * returned.  If pxEventGroupBuffer was NULL then NULL is returned.
 *
 * Example usage:
 * @code{c}
 * 	// StaticEventGroup_t is a publicly accessible structure that has the same
 * 	// size and alignment requirements as the real event group structure.  It is
 * 	// provided as a mechanism for applications to know the size of the event
 * 	// group (which is dependent on the architecture and configuration file
 * 	// settings) without breaking the strict data hiding policy by exposing the
 * 	// real event group internals.  This StaticEventGroup_t variable is passed
 * 	// into the xSemaphoreCreateEventGroupStatic() function and is used to store
 * 	// the event group's data structures
 * 	StaticEventGroup_t xEventGroupBuffer;
 *
 * 	// Create the event group without dynamically allocating any memory.
 * 	xEventGroup = xEventGroupCreateStatic( &xEventGroupBuffer );
 * @endcode
 */
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
    //静态 暂时不管
	EventGroupHandle_t xEventGroupCreateStatic( StaticEventGroup_t *pxEventGroupBuffer ) PRIVILEGED_FUNCTION;
#endif

/**
 * [Potentially] block to wait for one or more bits to be set within a
 * previously created event group.
 *
 * This function cannot be called from an interrupt.
 *
 * @param xEventGroup The event group in which the bits are being tested.  The
 * event group must have previously been created using a call to
 * xEventGroupCreate().
 *
 * @param uxBitsToWaitFor A bitwise value that indicates the bit or bits to test
 * inside the event group.  For example, to wait for bit 0 and/or bit 2 set
 * uxBitsToWaitFor to 0x05.  To wait for bits 0 and/or bit 1 and/or bit 2 set
 * uxBitsToWaitFor to 0x07.  Etc.
 *
 * @param xClearOnExit If xClearOnExit is set to pdTRUE then any bits within
 * uxBitsToWaitFor that are set within the event group will be cleared before
 * xEventGroupWaitBits() returns if the wait condition was met (if the function
 * returns for a reason other than a timeout).  If xClearOnExit is set to
 * pdFALSE then the bits set in the event group are not altered when the call to
 * xEventGroupWaitBits() returns.
 *
 * @param xWaitForAllBits If xWaitForAllBits is set to pdTRUE then
 * xEventGroupWaitBits() will return when either all the bits in uxBitsToWaitFor
 * are set or the specified block time expires.  If xWaitForAllBits is set to
 * pdFALSE then xEventGroupWaitBits() will return when any one of the bits set
 * in uxBitsToWaitFor is set or the specified block time expires.  The block
 * time is specified by the xTicksToWait parameter.
 *
 * @param xTicksToWait The maximum amount of time (specified in 'ticks') to wait
 * for one/all (depending on the xWaitForAllBits value) of the bits specified by
 * uxBitsToWaitFor to become set.
 *
 * @return 事件组的值在被等待的位被设置，或阻塞时间到期的值。 测试返回值以知道哪些位被设置。
 * 如果xEventGroupWaitBits()返回是因为它的超时时间过了，那么并非所有被等待的比特都会被设置。
 * 如果xEventGroupWaitBits()返回是因为它等待的位被设置了，那么在xClearOnExit参数被设置为pdTRUE的情况下，
 * 返回的值是事件组自动清除任何位之前的值
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
 *
 * 		// Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
 * 		// the event group.  Clear the bits before exiting.
 * 		uxBits = xEventGroupWaitBits(
 * 					xEventGroup,	// The event group being tested.
 * 					BIT_0 | BIT_4,	// The bits within the event group to wait for.
 * 					pdTRUE,			// BIT_0 and BIT_4 should be cleared before returning.
 * 					pdFALSE,		// Don't wait for both bits, either bit will do.
 * 					xTicksToWait );	// Wait a maximum of 100ms for either bit to be set.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// xEventGroupWaitBits() returned because both bits were set.
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// xEventGroupWaitBits() returned because just BIT_0 was set.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// xEventGroupWaitBits() returned because just BIT_4 was set.
 * 		}
 * 		else
 * 		{
 * 			// xEventGroupWaitBits() returned because xTicksToWait ticks passed
 * 			// without either BIT_0 or BIT_4 becoming set.
 * 		}
 *    }
 * @endcode{c}
 * \ingroup EventGroup
 */
EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait )
{
    rt_uint32_t bits = 0;
    rt_event_t event = (rt_event_t)xEventGroup;

    /* parameter check */
    RT_ASSERT(event != RT_NULL);
    RT_ASSERT(rt_object_get_type(&event->parent.parent) == RT_Object_Class_Event);
    rt_uint8_t option = ((xWaitForAllBits == pdTRUE)?RT_EVENT_FLAG_AND:RT_EVENT_FLAG_OR)|((xClearOnExit == pdTRUE)?RT_EVENT_FLAG_CLEAR:0x00);

    rt_event_recv(event, uxBitsToWaitFor, option, ((xTicksToWait==portMAX_DELAY)?RT_WAITING_FOREVER:xTicksToWait), &bits);
    return bits;
}

/**
 * Clear bits within an event group.  This function cannot be called from an
 * interrupt.
 *
 * @param xEventGroup The event group in which the bits are to be cleared.
 *
 * @param uxBitsToClear A bitwise value that indicates the bit or bits to clear
 * in the event group.  For example, to clear bit 3 only, set uxBitsToClear to
 * 0x08.  To clear bit 3 and bit 0 set uxBitsToClear to 0x09.
 *
 * @return The value of the event group before the specified bits were cleared.
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *
 * 		// Clear bit 0 and bit 4 in xEventGroup.
 * 		uxBits = xEventGroupClearBits(
 * 								xEventGroup,	// The event group being updated.
 * 								BIT_0 | BIT_4 );// The bits being cleared.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// Both bit 0 and bit 4 were set before xEventGroupClearBits() was
 * 			// called.  Both will now be clear (not set).
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// Bit 0 was set before xEventGroupClearBits() was called.  It will
 * 			// now be clear.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// Bit 4 was set before xEventGroupClearBits() was called.  It will
 * 			// now be clear.
 * 		}
 * 		else
 * 		{
 * 			// Neither bit 0 nor bit 4 were set in the first place.
 * 		}
 *    }
 * @endcode
 * \ingroup EventGroup
 */
// ipc内部函数 由于是static 故这里重新实现一遍
rt_inline rt_err_t rt_ipc_list_resume_all(rt_list_t *list)
{
    struct rt_thread *thread;
    register rt_ubase_t temp;

    /* wakeup all suspend threads */
    while (!rt_list_isempty(list))
    {
        /* disable interrupt */
        temp = rt_hw_interrupt_disable();

        /* get next suspend thread */
        thread = rt_list_entry(list->next, struct rt_thread, tlist);
        /* set error code to RT_ERROR */
        thread->error = -RT_ERROR;

        /*
         * resume thread
         * In rt_thread_resume function, it will remove current thread from
         * suspend list
         */
        rt_thread_resume(thread);

        /* enable interrupt */
        rt_hw_interrupt_enable(temp);
    }

    return RT_EOK;
}
EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear )
{
    //任务只有在等待事件发生（置一）时才会挂起 一旦置一则会立马调度
    //这里相当于手动清零事件
    rt_ubase_t level;
    rt_event_t event = (rt_event_t)xEventGroup;
    rt_uint32_t eventset = 0;

    /* parameter check */
    RT_ASSERT(event != RT_NULL);
    RT_ASSERT(rt_object_get_type(&event->parent.parent) == RT_Object_Class_Event);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /* resume all waiting thread */
    rt_ipc_list_resume_all(&event->parent.suspend_thread);

    eventset = event->set;
    /* init event set */
    event->set = 0;

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    // 中断中不进行调度
    if(rt_interrupt_get_nest() == 0)
        rt_schedule();

    return eventset;
}


/**
 * Set bits within an event group.
 * This function cannot be called from an interrupt.  xEventGroupSetBitsFromISR()
 * is a version that can be called from an interrupt.
 *
 * Setting bits in an event group will automatically unblock tasks that are
 * blocked waiting for the bits.
 *
 * @param xEventGroup The event group in which the bits are to be set.
 *
 * @param uxBitsToSet A bitwise value that indicates the bit or bits to set.
 * For example, to set bit 3 only, set uxBitsToSet to 0x08.  To set bit 3
 * and bit 0 set uxBitsToSet to 0x09.
 *
 * @return 调用 xEventGroupSetBits()返回时事件组的值。 有两个原因导致返回的值可能会清除 uxBitsToSet参数指定的位。
 * 首先，如果设置位的结果是等待位的任务离开阻塞状态，那么有可能位会被自动清除（参见 xEventGroupWaitBits()的 xClearBitOnExit参数）。
 * 其次，任何未被阻塞的（或其他 Ready状态）任务的优先级高于调用 xEventGroupSetBits()的任务的优先级，都会执行，并可能在调用 xEventGroupSetBits()返回之前改变事件组的值。
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *
 * 		// Set bit 0 and bit 4 in xEventGroup.
 * 		uxBits = xEventGroupSetBits(
 * 							xEventGroup,	// The event group being updated.
 * 							BIT_0 | BIT_4 );// The bits being set.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// Both bit 0 and bit 4 remained set when the function returned.
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// Bit 0 remained set when the function returned, but bit 4 was
 * 			// cleared.  It might be that bit 4 was cleared automatically as a
 * 			// task that was waiting for bit 4 was removed from the Blocked
 * 			// state.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// Bit 4 remained set when the function returned, but bit 0 was
 * 			// cleared.  It might be that bit 0 was cleared automatically as a
 * 			// task that was waiting for bit 0 was removed from the Blocked
 * 			// state.
 * 		}
 * 		else
 * 		{
 * 			// Neither bit 0 nor bit 4 remained set.  It might be that a task
 * 			// was waiting for both of the bits to be set, and the bits were
 * 			// cleared as the task left the Blocked state.
 * 		}
 *    }
 * @endcode{c}
 * \ingroup EventGroup
 */
EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet )
{
    //发送事件
    rt_event_t event = (rt_event_t)xEventGroup;
    rt_event_send(event, uxBitsToSet);
    return event->set;
}

/**
 * 原子化地设置事件组内的比特，然后等待同一事件组内的比特组合被设置。 这个功能通常用于同步多个任务，每个任务都要等待其他任务达到同步点后才能继续。
 *
 * This function cannot be used from an interrupt.
 *
 * The function will return before its block time expires if the bits specified
 * by the uxBitsToWait parameter are set, or become set within that time.  In
 * this case all the bits specified by uxBitsToWait will be automatically
 * cleared before the function returns.
 *
 * @param xEventGroup The event group in which the bits are being tested.  The
 * event group must have previously been created using a call to
 * xEventGroupCreate().
 *
 * @param uxBitsToSet The bits to set in the event group before determining
 * if, and possibly waiting for, all the bits specified by the uxBitsToWait
 * parameter are set.
 *
 * @param uxBitsToWaitFor A bitwise value that indicates the bit or bits to test
 * inside the event group.  For example, to wait for bit 0 and bit 2 set
 * uxBitsToWaitFor to 0x05.  To wait for bits 0 and bit 1 and bit 2 set
 * uxBitsToWaitFor to 0x07.  Etc.
 *
 * @param xTicksToWait The maximum amount of time (specified in 'ticks') to wait
 * for all of the bits specified by uxBitsToWaitFor to become set.
 *
 * @return The value of the event group at the time either the bits being waited
 * for became set, or the block time expired.  Test the return value to know
 * which bits were set.  If xEventGroupSync() returned because its timeout
 * expired then not all the bits being waited for will be set.  If
 * xEventGroupSync() returned because all the bits it was waiting for were
 * set then the returned value is the event group value before any bits were
 * automatically cleared.
 *
 * Example usage:
 * @code{c}
 *  // Bits used by the three tasks.
 *  #define TASK_0_BIT		( 1 << 0 )
 *  #define TASK_1_BIT		( 1 << 1 )
 *  #define TASK_2_BIT		( 1 << 2 )
 *
 *  #define ALL_SYNC_BITS ( TASK_0_BIT | TASK_1_BIT | TASK_2_BIT )
 *
 *  // Use an event group to synchronise three tasks.  It is assumed this event
 *  // group has already been created elsewhere.
 *  EventGroupHandle_t xEventBits;
 *
 *  void vTask0( void *pvParameters )
 *  {
 *  EventBits_t uxReturn;
 *  TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
 *
 * 	 for( ;; )
 * 	 {
 * 		// Perform task functionality here.
 *
 * 		// Set bit 0 in the event flag to note this task has reached the
 * 		// sync point.  The other two tasks will set the other two bits defined
 * 		// by ALL_SYNC_BITS.  All three tasks have reached the synchronisation
 * 		// point when all the ALL_SYNC_BITS are set.  Wait a maximum of 100ms
 * 		// for this to happen.
 * 		uxReturn = xEventGroupSync( xEventBits, TASK_0_BIT, ALL_SYNC_BITS, xTicksToWait );
 *
 * 		if( ( uxReturn & ALL_SYNC_BITS ) == ALL_SYNC_BITS )
 * 		{
 * 			// All three tasks reached the synchronisation point before the call
 * 			// to xEventGroupSync() timed out.
 * 		}
 * 	}
 *  }
 *
 *  void vTask1( void *pvParameters )
 *  {
 * 	 for( ;; )
 * 	 {
 * 		// Perform task functionality here.
 *
 * 		// Set bit 1 in the event flag to note this task has reached the
 * 		// synchronisation point.  The other two tasks will set the other two
 * 		// bits defined by ALL_SYNC_BITS.  All three tasks have reached the
 * 		// synchronisation point when all the ALL_SYNC_BITS are set.  Wait
 * 		// indefinitely for this to happen.
 * 		xEventGroupSync( xEventBits, TASK_1_BIT, ALL_SYNC_BITS, portMAX_DELAY );
 *
 * 		// xEventGroupSync() was called with an indefinite block time, so
 * 		// this task will only reach here if the syncrhonisation was made by all
 * 		// three tasks, so there is no need to test the return value.
 * 	 }
 *  }
 *
 *  void vTask2( void *pvParameters )
 *  {
 * 	 for( ;; )
 * 	 {
 * 		// Perform task functionality here.
 *
 * 		// Set bit 2 in the event flag to note this task has reached the
 * 		// synchronisation point.  The other two tasks will set the other two
 * 		// bits defined by ALL_SYNC_BITS.  All three tasks have reached the
 * 		// synchronisation point when all the ALL_SYNC_BITS are set.  Wait
 * 		// indefinitely for this to happen.
 * 		xEventGroupSync( xEventBits, TASK_2_BIT, ALL_SYNC_BITS, portMAX_DELAY );
 *
 * 		// xEventGroupSync() was called with an indefinite block time, so
 * 		// this task will only reach here if the syncrhonisation was made by all
 * 		// three tasks, so there is no need to test the return value.
 * 	}
 *  }
 *
 * @endcode
 * \ingroup EventGroup
 */
EventBits_t xEventGroupSync( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet, const EventBits_t uxBitsToWaitFor, TickType_t xTicksToWait )
{
    //其他地方暂未使用 暂时不管
    rt_kprintf("error : xEventGroupSync not implemented!!!\n\n");
    return 0;
}

/**
 * A version of xEventGroupGetBits() that can be called from an ISR.
 *
 * @param xEventGroup The event group being queried.
 *
 * @return The event group bits at the time xEventGroupGetBitsFromISR() was called.
 *
 * \ingroup EventGroup
 */
EventBits_t xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup )
{
    return xEventGroupClearBits(xEventGroup, 0);
}

/**
 *
 * Delete an event group that was previously created by a call to
 * xEventGroupCreate().  Tasks that are blocked on the event group will be
 * unblocked and obtain 0 as the event group's value.
 *
 * @param xEventGroup The event group being deleted.
 */
void vEventGroupDelete( EventGroupHandle_t xEventGroup )
{
    rt_event_delete((rt_event_t)xEventGroup);//仅适用于动态分配
}



