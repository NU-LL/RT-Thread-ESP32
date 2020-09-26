#include <rthw.h>
#include <rtthread.h>

#include <string.h>
#include <stdlib.h>
#include <sys/reent.h>
// #include <malloc.h>
#include "esp_heap_caps.h"


#if defined (RT_USING_HEAP) && defined (RT_USING_MEMESP32)

// extern void *heap_caps_malloc_default( size_t size );
// extern void *heap_caps_realloc_default( void *ptr, size_t size );
extern void *heap_caps_malloc( size_t size, uint32_t caps );
extern void *heap_caps_realloc( void *ptr, size_t size, int caps);
extern void *heap_caps_calloc( size_t n, size_t size, uint32_t caps);

/**
 * @ingroup SystemInit
 *
 * This function will init system heap
 *
 * @param begin_addr the beginning address of system page
 * @param end_addr the end address of system page
 */
void rt_system_heap_init(void *begin_addr, void *end_addr)
{
    //啥也不用干 idf内存管理组件会提前初始化
    return RT_EOK;
}

/**
 * @addtogroup MM
 */

/**@{*/

/**
 * This function will allocate a block from system heap memory.
 * - If the nbytes is less than zero,
 * or
 * - If there is no nbytes sized memory valid in system,
 * the RT_NULL is returned.
 *
 * @param size the size of memory to be allocated
 *
 * @return the allocated memory
 */
void *rt_malloc(rt_size_t size)
{
    // return heap_caps_malloc_default(size);
    // void *r = heap_caps_malloc( size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    // rt_kprintf("malloc:0x%08x size:%d\n", r, size);
    // return r;
    return heap_caps_malloc( size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
RTM_EXPORT(rt_malloc);

/**
 * This function will change the size of previously allocated memory block.
 *
 * @param ptr the previously allocated memory block
 * @param size the new size of memory block
 *
 * @return the allocated memory
 */
void *rt_realloc(void *ptr, rt_size_t size)
{
    // return heap_caps_realloc_default(ptr, size);
    return heap_caps_realloc(ptr, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
RTM_EXPORT(rt_realloc);

/**
 * This function will contiguously allocate enough space for count objects
 * that are size bytes of memory each and returns a pointer to the allocated
 * memory.
 *
 * The allocated memory is filled with bytes of value zero.
 *
 * @param count number of objects to allocate
 * @param size size of the objects to allocate
 *
 * @return pointer to allocated memory / NULL pointer if there is an error
 */
void *rt_calloc(rt_size_t count, rt_size_t size)
{
    // return _calloc_r(_REENT, count, size);
    return heap_caps_calloc(count, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
RTM_EXPORT(rt_calloc);

/**
 * This function will release the previous allocated memory block by rt_malloc.
 * The released memory block is taken back to system heap.
 *
 * @param ptr the address of memory which will be released
 */
void rt_free(void *ptr)
{
    heap_caps_free(ptr);
}
RTM_EXPORT(rt_free);

/**@}*/

#endif
