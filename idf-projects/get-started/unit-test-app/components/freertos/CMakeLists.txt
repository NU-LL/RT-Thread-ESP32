set(srcs
    "libcpu/xtensa/common/xtensa_context.S"
    "libcpu/xtensa/common/xtensa_init.c"
    "libcpu/xtensa/common/xtensa_intr.c"
    "libcpu/xtensa/common/xtensa_intr_asm.S"
    "libcpu/xtensa/common/xtensa_vector_defaults.S"
    "libcpu/xtensa/common/xtensa_vectors.S"
    "libcpu/xtensa/esp32/portasm.S"
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
    "src/timer.c"
    "freertos_wrapper/port.c"
    "freertos_wrapper/event_groups.c"
    "freertos_wrapper/list.c"
    "freertos_wrapper/queue.c"
    "freertos_wrapper/tasks.c"
    "freertos_wrapper/timers.c"
    # "finsh/cmd.c"
    # "finsh/msh.c"
    # "finsh/shell.c"
    )

# set(COMPONENT_ALIAS "freertos")
# set(COMPONENT_NAME "freertos")
# set(COMPONENT_TARGET "freertos")
# set(COMPONENT_LIBRARY "freertos")
# set(COMPONENT_ADD_LDFLAGS "-lrt-thread")

# get_property( MODEL_SEL GLOBAL PROPERTY MODEL_SEL )
# message( "BUILD_COMPONENTS=${BUILD_COMPONENTS} COMPONENT=${COMPONENT_NAME}" )

# app_trace is required by FreeRTOS headers only when CONFIG_SYSVIEW_ENABLE=y,
# but requirements can't depend on config options, so always require it.
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include ./
                    PRIV_INCLUDE_DIRS include/libc include/freertos libcpu/xtensa/common libcpu/xtensa/esp32 ./
                    LDFRAGMENTS linker.lf
                    REQUIRES app_trace
                    PRIV_REQUIRES soc)

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=uxTopUsedPriority")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=unused-value)

# set_source_files_properties(
#     freertos/semphr.h
#     PROPERTIES COMPILE_FLAGS
#     -Wno-error=unused-value
#     )

# set_source_files_properties(
#     tasks.c
#     event_groups.c
#     timers.c
#     queue.c
#     PROPERTIES COMPILE_DEFINITIONS
#     _ESP_FREERTOS_INTERNAL
#     )
