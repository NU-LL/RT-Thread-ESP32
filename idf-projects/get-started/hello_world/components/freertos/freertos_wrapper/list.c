#include <rthw.h>
#include <rtthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/list.h"


//初始化列表
void vListInitialise( List_t * const pxList )
{
    rt_kprintf("vListInitialise not implemented!!!\n\n");
}


//初始化列表项
void vListInitialiseItem( ListItem_t * const pxItem )
{
    rt_kprintf("vListInitialiseItem not implemented!!!\n\n");
}

//插入
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem )
{
    rt_kprintf("vListInsert not implemented!!!\n\n");
}


//插入最后
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem )
{
    rt_kprintf("vListInsertEnd not implemented!!!\n\n");
}

//移除
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove )
{
    rt_kprintf("uxListRemove not implemented!!!\n\n");
    return 0;
}


