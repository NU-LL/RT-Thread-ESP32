set(srcs
    "libcpu/xtensa/esp32/context_gcc.S"
    "libcpu/xtensa/esp32/cpuport.c"
    "board.c"
    "src/clock.c"
    "src/components.c"
    "src/cpu.c"
    "src/idle.c"
    "src/ipc.c"
    "src/irq.c"
    "src/kservice.c"
    "src/mem.c"
    "src/memesp32.c"
    "src/memheap.c"
    "src/mempool.c"
    "src/object.c"
    "src/scheduler.c"
    "src/slab.c"
    "src/thread.c"
    "freertos_wrapper/event_groups.c"
    "freertos_wrapper/list.c"
    "freertos_wrapper/queue.c"
    "freertos_wrapper/tasks.c"
    "freertos_wrapper/timers.c"
    )

# app_trace is required by FreeRTOS headers only when CONFIG_SYSVIEW_ENABLE=y,
# but requirements can't depend on config options, so always require it.
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
                    PRIV_INCLUDE_DIRS include/libc libcpu/xtensa/common libcpu/xtensa/esp32 .
                    LDFRAGMENTS linker.lf
                    REQUIRES app_trace
                    PRIV_REQUIRES soc)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=uxTopUsedPriority")

# set_source_files_properties(
#     tasks.c
#     event_groups.c
#     timers.c
#     queue.c
#     PROPERTIES COMPILE_DEFINITIONS
#     _ESP_FREERTOS_INTERNAL
#     )
