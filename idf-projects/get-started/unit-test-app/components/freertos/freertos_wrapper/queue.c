#include <rthw.h>
#include <rtthread.h>


#include <stdlib.h>
#include <string.h>

#include "esp32/rom/ets_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


struct QueueSet{
    rt_mq_t mq;
    //队列集合
    rt_mq_t *mq_buff;//存放队列指针的数组
    rt_uint16_t now;//现在空闲的数组的偏移
    rt_uint16_t max;//数组大小
};
typedef struct QueueSet *QueueSet_t;

/**
 * 最好使用宏xQueueSend()、xQueueSendToFront()和xQueueSendToBack()来代替直接调用这个函数
 *
 * 在队列中发送一个消息。 该消息是通过复制而不是引用的
 * 这个函数不能从中断服务例程中调用
 * 参见xQueueSendFromISR()以了解可在ISR中使用的替代方法
 *
 * @param xQueue The handle to the queue on which the item is to be posted.
 *
 * @param pvItemToQueue 一个指向要放在队列上的项目的指针，队列的项目大小在创建队列时就已经定义好了，
 * 所以这个字节数将从pvItemToQueue复制到队列存储区。 队列将容纳的项目的大小是在创建队列时定义的，
 * 所以这许多字节将从pvItemToQueue复制到队列存储区。
 * A pointer to the item that is to be placed on the
 * queue.  The size of the items the queue will hold was defined when the
 * queue was created, so this many bytes will be copied from pvItemToQueue
 * into the queue storage area.
 *
 * @param xTicksToWait 如果队列已经满了，任务应该阻止等待队列上的空闲空间的最大时间。
 * 如果将此值设置为 0，并且队列已满，则调用将立即返回。 时间是以 tick 周期来定义的， 因此如果需要的话， 应使用常量 portTICK_PERIOD_MS 来转换为实时时间。
 *
 * @param xCopyPosition 可以取值queueSEND_TO_BACK将项目放在队列的后面，或者queueSEND_TO_FRONT将项目放在队列的前面（对于高优先级的消息）
 *
 * @return pdTRUE if the item was successfully posted, otherwise errQUEUE_FULL.
 *
 * Example usage:
 * @code{c}
 *  struct AMessage
 *  {
 *  char ucMessageID;
 *  char ucData[ 20 ];
 *  } xMessage;
 *
 *  uint32_t ulVar = 10UL;
 *
 *  void vATask( void *pvParameters )
 *  {
 *  QueueHandle_t xQueue1, xQueue2;
 *  struct AMessage *pxMessage;
 *
 *  // Create a queue capable of containing 10 uint32_t values.
 *  xQueue1 = xQueueCreate( 10, sizeof( uint32_t ) );
 *
 *  // Create a queue capable of containing 10 pointers to AMessage structures.
 *  // These should be passed by pointer as they contain a lot of data.
 *  xQueue2 = xQueueCreate( 10, sizeof( struct AMessage * ) );
 *
 *  // ...
 *
 *  if( xQueue1 != 0 )
 *  {
 *      // Send an uint32_t.  Wait for 10 ticks for space to become
 *      // available if necessary.
 *      if( xQueueGenericSend( xQueue1, ( void * ) &ulVar, ( TickType_t ) 10, queueSEND_TO_BACK ) != pdPASS )
 *      {
 *          // Failed to post the message, even after 10 ticks.
 *      }
 *  }
 *
 *  if( xQueue2 != 0 )
 *  {
 *      // Send a pointer to a struct AMessage object.  Don't block if the
 *      // queue is already full.
 *      pxMessage = & xMessage;
 *      xQueueGenericSend( xQueue2, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
 *  }
 *
 *  // ... Rest of task code.
 *  }
 * @endcode
 * \ingroup QueueManagement
 */
BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition )
{
    rt_mq_t mq = (rt_mq_t) xQueue;



    // rt_mq_send_wait();//3.1.3 nano 版本中暂无该函数

    if(xCopyPosition == queueSEND_TO_BACK)
    {
        if(rt_mq_send(mq, pvItemToQueue, mq->msg_size) == RT_EOK)
            return pdPASS;
    }
    else if(xCopyPosition == queueSEND_TO_FRONT)
    {
        if(rt_mq_urgent(mq, pvItemToQueue, mq->msg_size) == RT_EOK)
            return pdPASS;
    }
    else if(xCopyPosition == queueOVERWRITE)
    {
        //仅用于长度为1的队列--所以队列要么空，要么满（xQueueOverwrite函数的实现）  这里不实现 可能会有bug   ！！需要实现 iic 会用到！！
        if(rt_mq_send(mq, pvItemToQueue, mq->msg_size) == RT_EOK)
            return pdPASS;
    }

    return errQUEUE_FULL;
}

/**
 * A version of xQueuePeek() that can be called from an interrupt service
 * routine (ISR).
 *
 * Receive an item from a queue without removing the item from the queue.
 * The item is received by copy so a buffer of adequate size must be
 * provided.  The number of bytes copied into the buffer was defined when
 * the queue was created.
 *
 * Successfully received items remain on the queue so will be returned again
 * by the next call, or a call to xQueueReceive().
 *
 * @param xQueue The handle to the queue from which the item is to be
 * received.
 *
 * @param pvBuffer Pointer to the buffer into which the received item will
 * be copied.
 *
 * @return pdTRUE if an item was successfully received from the queue,
 * otherwise pdFALSE.
 *
 * \ingroup QueueManagement
 */
BaseType_t xQueuePeekFromISR( QueueHandle_t xQueue,  void * const pvBuffer )
{
    //暂时不实现
    return pdTRUE;
}


/**
 * It is preferred that the macro xQueueReceive() be used rather than calling
 * this function directly.
 *
 * Receive an item from a queue.  The item is received by copy so a buffer of
 * adequate size must be provided.  The number of bytes copied into the buffer
 * was defined when the queue was created.
 *
 * This function must not be used in an interrupt service routine.  See
 * xQueueReceiveFromISR for an alternative that can.
 *
 * @param xQueue The handle to the queue from which the item is to be
 * received.
 *
 * @param pvBuffer Pointer to the buffer into which the received item will
 * be copied.
 *
 * @param xTicksToWait The maximum amount of time the task should block
 * waiting for an item to receive should the queue be empty at the time
 * of the call.	 The time is defined in tick periods so the constant
 * portTICK_PERIOD_MS should be used to convert to real time if this is required.
 * xQueueGenericReceive() will return immediately if the queue is empty and
 * xTicksToWait is 0.
 *
 * @param xJustPeek 当设置为 "true "时，从队列中接收的物品实际上不会从队列中删除--这意味着随后对xQueueReceive()的调用将返回相同的消息。
 * 当设置为false时，从队列中接收的项目也会从队列中移除。
 *
 * @return pdTRUE if an item was successfully received from the queue,
 * otherwise pdFALSE.
 *
 * Example usage:
 * @code{c}
 *  struct AMessage
 *  {
 * 	char ucMessageID;
 * 	char ucData[ 20 ];
 *  } xMessage;
 *
 *  QueueHandle_t xQueue;
 *
 *  // Task to create a queue and post a value.
 *  void vATask( void *pvParameters )
 *  {
 *  struct AMessage *pxMessage;
 *
 * 	// Create a queue capable of containing 10 pointers to AMessage structures.
 * 	// These should be passed by pointer as they contain a lot of data.
 * 	xQueue = xQueueCreate( 10, sizeof( struct AMessage * ) );
 * 	if( xQueue == 0 )
 * 	{
 * 		// Failed to create the queue.
 * 	}
 *
 * 	// ...
 *
 * 	// Send a pointer to a struct AMessage object.  Don't block if the
 * 	// queue is already full.
 * 	pxMessage = & xMessage;
 * 	xQueueSend( xQueue, ( void * ) &pxMessage, ( TickType_t ) 0 );
 *
 * 	// ... Rest of task code.
 *  }
 *
 *  // Task to receive from the queue.
 *  void vADifferentTask( void *pvParameters )
 *  {
 *  struct AMessage *pxRxedMessage;
 *
 * 	if( xQueue != 0 )
 * 	{
 * 		// Receive a message on the created queue.  Block for 10 ticks if a
 * 		// message is not immediately available.
 * 		if( xQueueGenericReceive( xQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) )
 * 		{
 * 			// pcRxedMessage now points to the struct AMessage variable posted
 * 			// by vATask.
 * 		}
 * 	}
 *
 * 	// ... Rest of task code.
 *  }
 * @endcode
 * \ingroup QueueManagement
 */
BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait, const BaseType_t xJustPeeking )
{
    rt_mq_t mq = (rt_mq_t) xQueue;

    //暂未实现 xJustPeeking == true的情况
    if(xJustPeeking == pdFALSE)
    {
        if(rt_mq_recv(mq, pvBuffer, mq->msg_size, xTicksToWait) == RT_EOK)
            return pdTRUE;
    }
    return pdFALSE;
}


/**
 * 返回队列中存储的信息数量。
 *
 * @param xQueue A handle to the queue being queried.
 *
 * @return The number of messages available in the queue.
 *
 * \ingroup QueueManagement
 */
UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue )
{
    // register rt_ubase_t temp;
    // UBaseType_t res;

    rt_mq_t mq = (rt_mq_t) xQueue;

    // /* disable interrupt */
    // temp = rt_hw_interrupt_disable();
    // res = mq->entry;
    // rt_hw_interrupt_enable(temp);

    return mq->entry;
}

/**
 * 返回队列中可用的空位数。 这等于在没有删除消息的情况下，在队列满之前可以发送到队列中的消息数量
 *
 * @param xQueue A handle to the queue being queried.
 *
 * @return The number of spaces available in the queue.
 *
 * \ingroup QueueManagement
 */
UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue )
{
    rt_mq_t mq = (rt_mq_t) xQueue;
    return (mq->max_msgs - mq->entry);
}

/**
 * Delete a queue - freeing all the memory allocated for storing of items
 * placed on the queue.
 *
 * @param xQueue A handle to the queue to be deleted.
 *
 * \ingroup QueueManagement
 */
void vQueueDelete( QueueHandle_t xQueue )
{
    rt_mq_t mq = (rt_mq_t) xQueue;
    rt_mq_delete(mq);//仅用于动态分配的 静态的暂时未支持
}


/**@{*/
/**
 * 最好使用宏xQueueSendFromISR()、xQueueSendToFrontFromISR()和xQueueSendToBackFromISR()来代替直接调用这个函数。
 * xQueueGiveFromISR() 与实际上不复制任何数据的信号量等效
 *
 * It is preferred that the macros xQueueSendFromISR(),
 * xQueueSendToFrontFromISR() and xQueueSendToBackFromISR() be used in place
 * of calling this function directly.  xQueueGiveFromISR() is an
 * equivalent for use by semaphores that don't actually copy any data.
 *
 * Post an item on a queue.  It is safe to use this function from within an
 * interrupt service routine.
 *
 * Items are queued by copy not reference so it is preferable to only
 * queue small items, especially when called from an ISR.  In most cases
 * it would be preferable to store a pointer to the item being queued.
 *
 * @param xQueue The handle to the queue on which the item is to be posted.
 *
 * @param pvItemToQueue A pointer to the item that is to be placed on the
 * queue.  The size of the items the queue will hold was defined when the
 * queue was created, so this many bytes will be copied from pvItemToQueue
 * into the queue storage area.
 *
 * @param[out] pxHigherPriorityTaskWoken 如果向队列发送消息导致一个任务运行，且该任务的优先级高于当前运行的任务，
 * 则xQueueGenericSendFromISR()将把*pxHigherPriorityTaskWoken设置为pdTRUE，
 * 如果xQueueGenericSendFromISR()将此值设置为pdTRUE，那么在中断退出之前应该请求上下文切换
 *
 * @param xCopyPosition Can take the value queueSEND_TO_BACK to place the
 * item at the back of the queue, or queueSEND_TO_FRONT to place the item
 * at the front of the queue (for high priority messages).
 *
 * @return pdTRUE if the data was successfully sent to the queue, otherwise
 * errQUEUE_FULL.
 *
 * Example usage for buffered IO (where the ISR can obtain more than one value
 * per call):
 * @code{c}
 *  void vBufferISR( void )
 *  {
 *  char cIn;
 *  BaseType_t xHigherPriorityTaskWokenByPost;
 *
 * 	// We have not woken a task at the start of the ISR.
 * 	xHigherPriorityTaskWokenByPost = pdFALSE;
 *
 * 	// Loop until the buffer is empty.
 * 	do
 * 	{
 * 		// Obtain a byte from the buffer.
 * 		cIn = portINPUT_BYTE( RX_REGISTER_ADDRESS );
 *
 * 		// Post each byte.
 * 		xQueueGenericSendFromISR( xRxQueue, &cIn, &xHigherPriorityTaskWokenByPost, queueSEND_TO_BACK );
 *
 * 	} while( portINPUT_BYTE( BUFFER_COUNT ) );
 *
 * 	// Now the buffer is empty we can switch context if necessary.  Note that the
 * 	// name of the yield function required is port specific.
 * 	if( xHigherPriorityTaskWokenByPost )
 * 	{
 * 		taskYIELD_YIELD_FROM_ISR();
 * 	}
 *  }
 * @endcode
 * \ingroup QueueManagement
 */
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue, const void * const pvItemToQueue, BaseType_t * const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition )
{
    //pxHigherPriorityTaskWoken（指针）: 如果发送消息后需要调度则将指向的地址中的值设置为 pdTRUE ，在调用该函数后会手动调度（rtt中会只要向队列中发送消息就会调度 无需设置为pdTRUE）
    *pxHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xQueueGenericSend(xQueue, pvItemToQueue, 0, xCopyPosition))
        return pdTRUE;
    else
        return errQUEUE_FULL;
}
// freertos中无论二进制信号量、计数信号量还是互斥量，它们都使用相同的获取和释放API函数
// 这里是释放（带中断保护）
// 所以这里需要区分是互斥量还是信号量
BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue, BaseType_t * const pxHigherPriorityTaskWoken )
{
    //pxHigherPriorityTaskWoken（指针）: 如果发送消息后需要调度则将指向的地址中的值设置为 pdTRUE ，在调用该函数后会手动调度（rtt中会只要向队列中发送消息就会调度 无需设置为pdTRUE）
    *pxHigherPriorityTaskWoken = pdFALSE;

    //互斥量和信号量第一个元素均为IPC的基类 struct rt_ipc_object，所以这里可以强转获得类型
    if(rt_object_get_type(&(((rt_sem_t)xQueue)->parent.parent)) == RT_Object_Class_Semaphore)
    {
        return (rt_sem_release((rt_sem_t)(xQueue)) == RT_EOK)?pdTRUE:errQUEUE_FULL;
    }
    // 互斥量不能在中断中使用
    // else if(rt_object_get_type(&(((rt_mutex_t)xQueue)->parent.parent)) == RT_Object_Class_Mutex)
    // {
    //     return (rt_mutex_release((rt_mutex_t)(xQueue)) == RT_EOK)?pdTRUE:errQUEUE_FULL;
    // }
    // 到了这里肯定是错的
    rt_kprintf("type error!!! code: %d\n", ((rt_sem_t)xQueue)->parent.parent.type);
    RT_ASSERT(0);
    return pdFALSE;
}
// freertos中无论二进制信号量、计数信号量还是互斥量，它们都使用相同的获取和释放API函数
// 这里是获取（带中断保护）
// 所以这里需要区分是互斥量还是信号量
BaseType_t xQueueTakeFromISR( QueueHandle_t xQueue, BaseType_t * const pxHigherPriorityTaskWoken )
{
    //pxHigherPriorityTaskWoken（指针）: 如果发送消息后需要调度则将指向的地址中的值设置为 pdTRUE ，在调用该函数后会手动调度（rtt中会只要向队列中发送消息就会调度 无需设置为pdTRUE）
    *pxHigherPriorityTaskWoken = pdFALSE;

    //互斥量和信号量第一个元素均为IPC的基类 struct rt_ipc_object，所以这里可以强转获得类型
    if(rt_object_get_type(&(((rt_sem_t)xQueue)->parent.parent)) == RT_Object_Class_Semaphore)
    {
        return (rt_sem_trytake((rt_sem_t)(xQueue))==RT_EOK)?pdTRUE:pdFALSE;
    }
    // 互斥量不能在中断中使用
    // else if(rt_object_get_type(&(((rt_mutex_t)xQueue)->parent.parent)) == RT_Object_Class_Mutex)
    // {
    //     return (rt_mutex_trytake((rt_mutex_t)(xQueue)) == RT_EOK)?pdTRUE:pdFALSE;
    // }
    // 到了这里肯定是错的
    rt_kprintf("type error!!! code: %d\n", ((rt_sem_t)xQueue)->parent.parent.type);
    RT_ASSERT(0);
    return pdFALSE;
}
/**@}*/

/**
 * Receive an item from a queue.  It is safe to use this function from within an
 * interrupt service routine.
 *
 * @param xQueue The handle to the queue from which the item is to be
 * received.
 *
 * @param pvBuffer Pointer to the buffer into which the received item will
 * be copied.
 *
 * @param[out] pxHigherPriorityTaskWoken A task may be blocked waiting for space to become
 * available on the queue.  If xQueueReceiveFromISR causes such a task to
 * unblock *pxTaskWoken will get set to pdTRUE, otherwise *pxTaskWoken will
 * remain unchanged.
 *
 * @return pdTRUE if an item was successfully received from the queue,
 * otherwise pdFALSE.
 *
 * Example usage:
 * @code{c}
 *  QueueHandle_t xQueue;
 *
 *  // Function to create a queue and post some values.
 *  void vAFunction( void *pvParameters )
 *  {
 *  char cValueToPost;
 *  const TickType_t xTicksToWait = ( TickType_t )0xff;
 *
 * 	// Create a queue capable of containing 10 characters.
 * 	xQueue = xQueueCreate( 10, sizeof( char ) );
 * 	if( xQueue == 0 )
 * 	{
 * 		// Failed to create the queue.
 * 	}
 *
 * 	// ...
 *
 * 	// Post some characters that will be used within an ISR.  If the queue
 * 	// is full then this task will block for xTicksToWait ticks.
 * 	cValueToPost = 'a';
 * 	xQueueSend( xQueue, ( void * ) &cValueToPost, xTicksToWait );
 * 	cValueToPost = 'b';
 * 	xQueueSend( xQueue, ( void * ) &cValueToPost, xTicksToWait );
 *
 * 	// ... keep posting characters ... this task may block when the queue
 * 	// becomes full.
 *
 * 	cValueToPost = 'c';
 * 	xQueueSend( xQueue, ( void * ) &cValueToPost, xTicksToWait );
 *  }
 *
 *  // ISR that outputs all the characters received on the queue.
 *  void vISR_Routine( void )
 *  {
 *  BaseType_t xTaskWokenByReceive = pdFALSE;
 *  char cRxedChar;
 *
 * 	while( xQueueReceiveFromISR( xQueue, ( void * ) &cRxedChar, &xTaskWokenByReceive) )
 * 	{
 * 		// A character was received.  Output the character now.
 * 		vOutputCharacter( cRxedChar );
 *
 * 		// If removing the character from the queue woke the task that was
 * 		// posting onto the queue cTaskWokenByReceive will have been set to
 * 		// pdTRUE.  No matter how many times this loop iterates only one
 * 		// task will be woken.
 * 	}
 *
 * 	if( cTaskWokenByPost != ( char ) pdFALSE;
 * 	{
 * 		taskYIELD ();
 * 	}
 *  }
 * @endcode
 * \ingroup QueueManagement
 */
BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue, void * const pvBuffer, BaseType_t * const pxHigherPriorityTaskWoken )
{
    //从队列中获得消息
    rt_mq_t mq = (rt_mq_t) xQueue;

    *pxHigherPriorityTaskWoken = pdFALSE;
    if(RT_EOK == rt_mq_recv(mq, pvBuffer, mq->msg_size, 0))
        return pdTRUE;
    return pdFALSE;
}

/**@{*/
/**
 * 用于查询队列的实用程序，可以在ISR中安全使用。 这些实用程序只能在ISR内或在关键部分使用。
 */
BaseType_t xQueueIsQueueEmptyFromISR( QueueHandle_t xQueue )
{
    rt_mq_t mq = (rt_mq_t) xQueue;

    if(mq->entry)
        return pdFALSE;
    else
        return pdTRUE;
}


BaseType_t xQueueIsQueueFullFromISR( QueueHandle_t xQueue )
{
    rt_mq_t mq = (rt_mq_t) xQueue;

    if(mq->entry >= mq->max_msgs)
        return pdTRUE;
    else
        return pdFALSE;
}

UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue )
{
    rt_mq_t mq = (rt_mq_t) xQueue;
    return mq->entry;
}

/**@}*/

/** @cond */
/**
 * xQueueAltGenericSend()是 xQueueGenericSend()的一个替代版本。
 * 同样， xQueueAltGenericReceive()也是 xQueueGenericReceive()的替代版本。
 *
 * 实现替代性(Alt)API的源代码要简单得多，因为它在一个关键部分内执行一切。
 * 这是许多其他RTOS所采取的方法，但FreeRTOS.org也有首选的全功能API。
 * 完全特色的API有更复杂的代码，需要更长的时间来执行，但对关键部分的使用要少得多。
 * 因此替代API牺牲了中断响应性来获得执行速度，而全功能API则牺牲了执行速度来保证更好的中断响应性
 */

#if ( configUSE_ALTERNATIVE_API == 1 )
	BaseType_t xQueueAltGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, BaseType_t xCopyPosition )
	{
        //暂时不实现 宏未启用
    }
#endif /* configUSE_ALTERNATIVE_API */

#if ( configUSE_ALTERNATIVE_API == 1 )

	BaseType_t xQueueAltGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait, BaseType_t xJustPeeking )
	{
        //暂时不实现 宏未启用
    }
#endif /* configUSE_ALTERNATIVE_API */

/*
 * The functions defined above are for passing data to and from tasks.  The
 * functions below are the equivalents for passing data to and from
 * co-routines.
 *
 * These functions are called from the co-routine macro implementation and
 * should not be called directly from application code.  Instead use the macro
 * wrappers defined within croutine.h.
 */
#if ( configUSE_CO_ROUTINES == 1 )
    //暂时不实现 宏未启用
	BaseType_t xQueueCRSendFromISR( QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t xCoRoutinePreviouslyWoken )
	{
    }
    BaseType_t xQueueCRReceiveFromISR( QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxCoRoutineWoken )
	{
    }
    BaseType_t xQueueCRSend( QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait )
	{
    }
    BaseType_t xQueueCRReceive( QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait )
	{
    }
#endif /* configUSE_CO_ROUTINES */

/*
 * 仅供内部使用。 使用 xSemaphoreCreateMutex()、 xSemaphoreCreateCounting()或 xSemaphoreGetMutexHolder()代替直接调用这些函数。
 */
#if( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
	QueueHandle_t xQueueCreateMutex( const uint8_t ucQueueType )//创建互斥量
	{
        //宏 xSemaphoreCreateMutex xSemaphoreCreateRecursiveMutex 的实现
        static rt_uint16_t idx = 0;
        char namebuf[16] = {0};
        sprintf( namebuf, "mutex_%02d", idx);
        idx++;
        return rt_mutex_create(namebuf, RT_IPC_FLAG_FIFO);
    }
#endif /* configUSE_MUTEXES */
#if( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
	QueueHandle_t xQueueCreateMutexStatic( const uint8_t ucQueueType, StaticQueue_t *pxStaticQueue )//创建静态互斥量
	{
        //宏 xSemaphoreCreateMutexStatic 的实现

	}
#endif /* configUSE_MUTEXES */
#if( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
	QueueHandle_t xQueueCreateCountingSemaphore( const UBaseType_t uxMaxCount, const UBaseType_t uxInitialCount )//创建信号量
	{
        //宏 xSemaphoreCreateCounting 的实现
        static rt_uint16_t idx = 0;
        char namebuf[16] = {0};
        sprintf( namebuf, "sem_%02d", idx);
        idx++;
        return rt_sem_create(namebuf, uxMaxCount-uxInitialCount, RT_IPC_FLAG_FIFO);//最大值-初值
    }
#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
#if( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
	QueueHandle_t xQueueCreateCountingSemaphoreStatic( const UBaseType_t uxMaxCount, const UBaseType_t uxInitialCount, StaticQueue_t *pxStaticQueue )//创建静态信号量
	{
        //宏 xSemaphoreCreateCountingStatic 的实现

    }
#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */

#if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )
//如果xMutex确实是互斥类型的信号量，返回当前的互斥量的(mutex)持有者。如果xMutex不是mutex类型的信号量，或者mutex是可用的（不是由任务持有），返回NULL。
//注意：这是确定调用任务是否是mutex持有者的好方法，但不是确定mutex持有者身份的好方法，因为在函数退出和返回测试值之间，持有者可能会改变。
	void* xQueueGetMutexHolder( QueueHandle_t xSemaphore )//获得互斥量
	{
        //宏 xSemaphoreGetMutexHolder 的实现
        rt_mutex_t mutex = (rt_mutex_t) xSemaphore;
        return mutex->owner;
    }
#endif


// freertos中无论二进制信号量、计数信号量还是互斥量，它们都使用相同的获取和释放API函数
// 这里是释放（不带中断保护）
// 所以这里需要区分是互斥量还是信号量
BaseType_t xsemaphore_give(QueueHandle_t xSemaphore)
{
    //互斥量和信号量第一个元素均为IPC的基类 struct rt_ipc_object，所以这里可以强转获得类型
    if(rt_object_get_type(&(((rt_sem_t)xSemaphore)->parent.parent)) == RT_Object_Class_Semaphore)
    {
        return (rt_sem_release((rt_sem_t)(xSemaphore)) == RT_EOK)?pdTRUE:pdFALSE;
    }else if(rt_object_get_type(&(((rt_mutex_t)xSemaphore)->parent.parent)) == RT_Object_Class_Mutex)
    {
        return (rt_mutex_release((rt_mutex_t)(xSemaphore)) == RT_EOK)?pdTRUE:pdFALSE;
    }
    // 到了这里肯定是错的
    rt_kprintf("type error!!! code: %d\n", ((rt_sem_t)xSemaphore)->parent.parent.type);
    RT_ASSERT(0);
    return pdFALSE;
}
// freertos中无论二进制信号量、计数信号量还是互斥量，它们都使用相同的获取和释放API函数
// 这里是获取（不带中断保护）
// 所以这里需要区分是互斥量还是信号量
BaseType_t xsemaphore_take(QueueHandle_t xSemaphore, TickType_t xBlockTime)
{
    //互斥量和信号量第一个元素均为IPC的基类 struct rt_ipc_object，所以这里可以强转获得类型
    if(rt_object_get_type(&(((rt_sem_t)xSemaphore)->parent.parent)) == RT_Object_Class_Semaphore)
    {
        return (rt_sem_take((rt_sem_t)xSemaphore, xBlockTime)==RT_EOK)?pdTRUE:pdFALSE;
    }else if(rt_object_get_type(&(((rt_mutex_t)xSemaphore)->parent.parent)) == RT_Object_Class_Mutex)
    {
        return (rt_mutex_take((rt_mutex_t)(xSemaphore), xBlockTime) == RT_EOK)?pdTRUE:pdFALSE;
    }
    // 到了这里肯定是错的
    rt_kprintf("type error!!! code: %d\n", ((rt_sem_t)xSemaphore)->parent.parent.type);
    RT_ASSERT(0);
    return pdFALSE;
}

/*
 * For internal use only.  Use xSemaphoreTakeMutexRecursive() or
 * xSemaphoreGiveMutexRecursive() instead of calling these functions directly.
 */
#if ( configUSE_RECURSIVE_MUTEXES == 1 )
//用于递归地获取或 "获取 "一个mutex类型的信号体。该mutex必须是在调用 xSemaphoreCreateRecursiveMutex()之前创建的。
//一个递归使用的mutex可以被所有者反复 "获取"。直到所有者对每次成功的 "获取 "请求调用 xSemaphoreGiveRecursive()后，
//该mutex才会再次可用。 例如，如果一个任务成功地 "拿走 "了同一个mutex 5次，那么在mutex "释放"5次之前，该mutex不会再被其他任务使用。
	BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex, TickType_t xTicksToWait )
	{
        //宏 xSemaphoreTakeRecursive 的实现
        //获得递归互斥量 rtt中互斥量支持递归
        rt_mutex_t mutex = (rt_mutex_t)xMutex;
        if(rt_mutex_take(mutex, xTicksToWait) == RT_EOK)
            return pdTRUE;
        return pdFALSE;
    }
    BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex )
	{
        //宏 xSemaphoreGiveRecursive 的实现
        //释放递归互斥量 rtt中互斥量支持递归
        rt_mutex_t mutex = (rt_mutex_t)xMutex;
        if(rt_mutex_release(mutex)== RT_EOK)
            return pdTRUE;
        return pdFALSE;
    }
#endif /* configUSE_RECURSIVE_MUTEXES */


/**
 * The registry is provided as a means for kernel aware debuggers to
 * locate queues, semaphores and mutexes.  Call vQueueAddToRegistry() add
 * a queue, semaphore or mutex handle to the registry if you want the handle
 * to be available to a kernel aware debugger.  If you are not using a kernel
 * aware debugger then this function can be ignored.
 *
 * configQUEUE_REGISTRY_SIZE defines the maximum number of handles the
 * registry can hold.  configQUEUE_REGISTRY_SIZE must be greater than 0
 * within FreeRTOSConfig.h for the registry to be available.  Its value
 * does not effect the number of queues, semaphores and mutexes that can be
 * created - just the number that the registry can hold.
 *
 * @param xQueue The handle of the queue being added to the registry.  This
 * is the handle returned by a call to xQueueCreate().  Semaphore and mutex
 * handles can also be passed in here.
 *
 * @param pcName The name to be associated with the handle.  This is the
 * name that the kernel aware debugger will display.  The queue registry only
 * stores a pointer to the string - so the string must be persistent (global or
 * preferably in ROM/Flash), not on the stack.
 */
#if configQUEUE_REGISTRY_SIZE > 0
	void vQueueAddToRegistry( QueueHandle_t xQueue, const char *pcName ) PRIVILEGED_FUNCTION; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
#endif

/**
 * The registry is provided as a means for kernel aware debuggers to
 * locate queues, semaphores and mutexes.  Call vQueueAddToRegistry() add
 * a queue, semaphore or mutex handle to the registry if you want the handle
 * to be available to a kernel aware debugger, and vQueueUnregisterQueue() to
 * remove the queue, semaphore or mutex from the register.  If you are not using
 * a kernel aware debugger then this function can be ignored.
 *
 * @param xQueue The handle of the queue being removed from the registry.
 */
#if configQUEUE_REGISTRY_SIZE > 0
	void vQueueUnregisterQueue( QueueHandle_t xQueue ) PRIVILEGED_FUNCTION;
#endif

/**
 * @note This function has been back ported from FreeRTOS v9.0.0
 *
 * The queue registry is provided as a means for kernel aware debuggers to
 * locate queues, semaphores and mutexes.  Call pcQueueGetName() to look
 * up and return the name of a queue in the queue registry from the queue's
 * handle.
 *
 * @param xQueue The handle of the queue the name of which will be returned.
 * @return If the queue is in the registry then a pointer to the name of the
 * queue is returned.  If the queue is not in the registry then NULL is
 * returned.
 */
#if( configQUEUE_REGISTRY_SIZE > 0 )
	const char *pcQueueGetName( QueueHandle_t xQueue ) PRIVILEGED_FUNCTION; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
#endif

/**
 * 用于使用动态内存分配来创建一个队列的通用函数。 这个函数被其他函数和宏调用，这些函数和宏创建了其他以队列结构为基础的 RTOS 对象
 */
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
    QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType )//长度 大小 类型(queueQUEUE_TYPE_BASE 调试时会用到 暂未实现)
	{
        static rt_uint16_t idx = 0;
        char namebuf[16] = {0};
        sprintf( namebuf, "mq_%02d", idx);
        idx++;
        return rt_mq_create(namebuf, (rt_size_t)uxItemSize, (rt_size_t)uxQueueLength, RT_IPC_FLAG_FIFO);
    }
#endif

/**
 * Generic version of the function used to creaet a queue using dynamic memory
 * allocation.  This is called by other functions and macros that create other
 * RTOS objects that use the queue structure as their base.
 */
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
	QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, StaticQueue_t *pxStaticQueue, const uint8_t ucQueueType )
    {
        static rt_uint16_t idx = 0;
        char namebuf[16] = {0};
        sprintf( namebuf, "s_mq_%02d", idx);
        idx++;
        rt_mq_init(pxStaticQueue, namebuf, (void *)pucQueueStorage, (rt_size_t)uxItemSize, (rt_size_t)uxQueueLength, RT_IPC_FLAG_FIFO);//pxStaticQueue应该重新实现 这里有问题 暂时不管
    }
#endif

/**
 * Queue sets provide a mechanism to allow a task to block (pend) on a read
 * operation from multiple queues or semaphores simultaneously.
 *
 * See FreeRTOS/Source/Demo/Common/Minimal/QueueSet.c for an example using this
 * function.
 *
 * A queue set must be explicitly created using a call to xQueueCreateSet()
 * before it can be used.  Once created, standard FreeRTOS queues and semaphores
 * can be added to the set using calls to xQueueAddToSet().
 * xQueueSelectFromSet() is then used to determine which, if any, of the queues
 * or semaphores contained in the set is in a state where a queue read or
 * semaphore take operation would be successful.
 *
 * Note 1:  See the documentation on http://wwwFreeRTOS.org/RTOS-queue-sets.html
 * for reasons why queue sets are very rarely needed in practice as there are
 * simpler methods of blocking on multiple objects.
 *
 * Note 2:  Blocking on a queue set that contains a mutex will not cause the
 * mutex holder to inherit the priority of the blocked task.
 *
 * Note 3:  An additional 4 bytes of RAM is required for each space in a every
 * queue added to a queue set.  Therefore counting semaphores that have a high
 * maximum count value should not be added to a queue set.
 *
 * Note 4:  A receive (in the case of a queue) or take (in the case of a
 * semaphore) operation must not be performed on a member of a queue set unless
 * a call to xQueueSelectFromSet() has first returned a handle to that set member.
 *
 * @param uxEventQueueLength Queue sets store events that occur on
 * the queues and semaphores contained in the set.  uxEventQueueLength specifies
 * the maximum number of events that can be queued at once.  To be absolutely
 * certain that events are not lost uxEventQueueLength should be set to the
 * total sum of the length of the queues added to the set, where binary
 * semaphores and mutexes have a length of 1, and counting semaphores have a
 * length set by their maximum count value.  Examples:
 *  + If a queue set is to hold a queue of length 5, another queue of length 12,
 *    and a binary semaphore, then uxEventQueueLength should be set to
 *    (5 + 12 + 1), or 18.
 *  + If a queue set is to hold three binary semaphores then uxEventQueueLength
 *    should be set to (1 + 1 + 1 ), or 3.
 *  + If a queue set is to hold a counting semaphore that has a maximum count of
 *    5, and a counting semaphore that has a maximum count of 3, then
 *    uxEventQueueLength should be set to (5 + 3), or 8.
 *
 * @return If the queue set is created successfully then a handle to the created
 * queue set is returned.  Otherwise NULL is returned.
 */
#if( ( configUSE_QUEUE_SETS == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
	QueueSetHandle_t xQueueCreateSet( const UBaseType_t uxEventQueueLength )
	{
        QueueSetHandle_t pxQueue;
        QueueSet_t Queue_set = rt_malloc(sizeof(struct QueueSet));

        // pxQueue = xQueueGenericCreate( uxEventQueueLength, sizeof( Queue_t * ), queueQUEUE_TYPE_SET );
        pxQueue = xQueueGenericCreate( uxEventQueueLength, sizeof(rt_mq_t), queueQUEUE_TYPE_SET );//申请 rt_mq_t

        Queue_set->mq = pxQueue;
        Queue_set->mq_buff = rt_malloc(uxEventQueueLength*sizeof(rt_mq_t));
        Queue_set->max = uxEventQueueLength;
        Queue_set->now = 0;

        return Queue_set;
	}
#endif /* configUSE_QUEUE_SETS */

/**
 * 将队列或信号量添加到之前通过调用 xQueueCreateSet()创建的队列集中
 *
 * See FreeRTOS/Source/Demo/Common/Minimal/QueueSet.c for an example using this
 * function.
 *
 * Note 1:  A receive (in the case of a queue) or take (in the case of a
 * semaphore) operation must not be performed on a member of a queue set unless
 * a call to xQueueSelectFromSet() has first returned a handle to that set member.
 *
 * @param xQueueOrSemaphore 被添加到队列集的队列或信号量的句柄（投向QueueSetMemberHandle_t类型）
 *
 * @param xQueueSet 队列或信号量被添加到的队列集的句柄
 *
 * @return 如果队列或信号量被成功添加到队列集中，则返回pdPASS。 如果由于队列已经是另一个队列的成员而无法成功添加到队列集中，则返回pdFAIL
 * If the queue or semaphore was successfully added to the queue set
 * then pdPASS is returned.  If the queue could not be successfully added to the
 * queue set because it is already a member of a different queue set then pdFAIL
 * is returned.
 */
#if ( configUSE_QUEUE_SETS == 1 )
	BaseType_t xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet )
	{
        rt_mq_t mq = (rt_mq_t) xQueueOrSemaphore;//待插入队列
        QueueSet_t Queue_set = (QueueSet_t)xQueueSet;//队列集
        rt_uint16_t idx = 0;
        //遍历 查找插入的队列是否有重复
        for(idx = 0; idx < Queue_set->now; idx++)
        {
            if(mq == Queue_set->mq_buff[idx])
                return pdFAIL;//有重复
        }
        if(Queue_set->now >= Queue_set->max)
        {
            // Queue_set->now = Queue_set->max-1;
            return pdFAIL;//集合已满
        }
        //插入
        Queue_set->mq_buff[Queue_set->now] = mq;
        //移动 now
        Queue_set->now++;

        return pdPASS;
	}
#endif /* configUSE_QUEUE_SETS */

/**
 * Removes a queue or semaphore from a queue set.  A queue or semaphore can only
 * be removed from a set if the queue or semaphore is empty.
 *
 * See FreeRTOS/Source/Demo/Common/Minimal/QueueSet.c for an example using this
 * function.
 *
 * @param xQueueOrSemaphore The handle of the queue or semaphore being removed
 * from the queue set (cast to an QueueSetMemberHandle_t type).
 *
 * @param xQueueSet The handle of the queue set in which the queue or semaphore
 * is included.
 *
 * @return If the queue or semaphore was successfully removed from the queue set
 * then pdPASS is returned.  If the queue was not in the queue set, or the
 * queue (or semaphore) was not empty, then pdFAIL is returned.
 */
#if ( configUSE_QUEUE_SETS == 1 )
	BaseType_t xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet )//从队列集中移除
	{
        rt_mq_t mq = (rt_mq_t) xQueueOrSemaphore;//待插入队列
        QueueSet_t Queue_set = (QueueSet_t)xQueueSet;//队列集
        rt_uint16_t idx = 0;

        if(mq == RT_NULL)
            return pdFAIL;//插入队列为空

        if(Queue_set->now == 0)
            return pdFAIL;//队列集为空

        //开始移除
        //遍历 查找插入的队列是否有重复
        for(idx = 0; idx < Queue_set->now; idx++)
        {
            if(mq == Queue_set->mq_buff[idx])//找到
            {
                // Queue_set->mq_buff[idx] = RT_NULL;//移除
                //移动 now
                Queue_set->now--;
                //若此时 now 处不为空，则需要将次队列填到之前为删除的位置上
                if(idx != Queue_set->now)
                    Queue_set->mq_buff[idx] = Queue_set->mq_buff[Queue_set->now];//移除 idx 处队列 此时now处肯定还有值，但是不用管
                // else
                //     Queue_set->mq_buff[idx] = RT_NULL;//
                return pdPASS;
            }
        }
        //未找到
        return pdFAIL;
    }
#endif /* configUSE_QUEUE_SETS */

/**
 * xQueueSelectFromSet() selects from the members of a queue set a queue or
 * semaphore that either contains data (in the case of a queue) or is available
 * to take (in the case of a semaphore).  xQueueSelectFromSet() effectively
 * allows a task to block (pend) on a read operation on all the queues and
 * semaphores in a queue set simultaneously.
 *
 * See FreeRTOS/Source/Demo/Common/Minimal/QueueSet.c for an example using this
 * function.
 *
 * Note 1:  See the documentation on http://wwwFreeRTOS.org/RTOS-queue-sets.html
 * for reasons why queue sets are very rarely needed in practice as there are
 * simpler methods of blocking on multiple objects.
 *
 * Note 2:  Blocking on a queue set that contains a mutex will not cause the
 * mutex holder to inherit the priority of the blocked task.
 *
 * Note 3:  A receive (in the case of a queue) or take (in the case of a
 * semaphore) operation must not be performed on a member of a queue set unless
 * a call to xQueueSelectFromSet() has first returned a handle to that set member.
 *
 * @param xQueueSet The queue set on which the task will (potentially) block.
 *
 * @param xTicksToWait The maximum time, in ticks, that the calling task will
 * remain in the Blocked state (with other tasks executing) to wait for a member
 * of the queue set to be ready for a successful queue read or semaphore take
 * operation.
 *
 * @return xQueueSelectFromSet()将返回包含数据的队列集中的队列句柄(投向QueueSetMemberHandle_t类型)，
 * 或包含在队列集中的信号量句柄(投向QueueSetMemberHandle_t类型)，
 * 该信号体是可用的，如果在指定的块时间到期前没有该队列或信号体存在，则返回NULL
 *
 * xQueueSelectFromSet() will return the handle of a queue (cast to
 * a QueueSetMemberHandle_t type) contained in the queue set that contains data,
 * or the handle of a semaphore (cast to a QueueSetMemberHandle_t type) contained
 * in the queue set that is available, or NULL if no such queue or semaphore
 * exists before before the specified block time expires.
 */
#if ( configUSE_QUEUE_SETS == 1 )
	QueueSetMemberHandle_t xQueueSelectFromSet( QueueSetHandle_t xQueueSet, TickType_t const xTicksToWait )
	{
        // rt_mq_t mq = (rt_mq_t) xQueueOrSemaphore;//待插入队列
        QueueSetMemberHandle_t xReturn = NULL;
        rt_mq_t mq = RT_NULL;
        QueueSet_t Queue_set = (QueueSet_t)xQueueSet;//队列集
        rt_uint16_t idx = 0;
        rt_int32_t times = xTicksToWait;

        if(Queue_set->now == 0)
            return NULL;

        //如果有队列由数据
        do
        {
            for(idx = 0; idx < Queue_set->now; idx++)
            {
                mq = Queue_set->mq_buff[idx];
                if(mq->entry)
                    return mq;//返回有消息的队列
            }
            rt_thread_delay(10);
            times = times - 10;
        }while(times >= 10);
        //超时 或者 没有符合的队列
        return NULL;
	}
#endif /* configUSE_QUEUE_SETS */

/**
 * A version of xQueueSelectFromSet() that can be used from an ISR.
 */
#if ( configUSE_QUEUE_SETS == 1 )
    QueueSetMemberHandle_t xQueueSelectFromSetFromISR( QueueSetHandle_t xQueueSet )
    {
        // rt_mq_t mq = (rt_mq_t) xQueueOrSemaphore;//待插入队列
        QueueSetMemberHandle_t xReturn = NULL;
        rt_mq_t mq = RT_NULL;
        QueueSet_t Queue_set = (QueueSet_t)xQueueSet;//队列集
        rt_uint16_t idx = 0;

        if(Queue_set->now == 0)
            return NULL;

        //如果有队列由数据
        for(idx = 0; idx < Queue_set->now; idx++)
        {
            mq = Queue_set->mq_buff[idx];
            if(mq->entry)
                return mq;//返回有消息的队列
        }
        //没有符合的队列
        return NULL;
    }
#endif /* configUSE_QUEUE_SETS */

/**
 * 如果队列被成功重置，则返回pdPASS。如果队列不能被重置，因为队列上有任务阻塞，等待从队列接收或发送到队列，则返回pdFAIL。
 *
 * @param xQueue The queue to reset
 * @return always returns pdPASS
 */
BaseType_t xQueueGenericReset( QueueHandle_t xQueue, BaseType_t xNewQueue )//pdFALSE
{
    rt_mq_t mq = (rt_mq_t) xQueue;
    if(RT_EOK == rt_mq_control(mq, RT_IPC_CMD_RESET, RT_NULL))
        return pdPASS;
    return pdFAIL;
}


