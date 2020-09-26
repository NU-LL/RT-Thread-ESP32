#
# Component Makefile
#

COMPONENT_ADD_LDFLAGS += -Wl,--undefined=uxTopUsedPriority
COMPONENT_ADD_INCLUDEDIRS := include ./
COMPONENT_PRIV_INCLUDEDIRS := include/libc include/freertos libcpu/xtensa/common libcpu/xtensa/esp32 ./


# freertos/semphr.o: CFLAGS += -Wno-error=unused-value


# tasks.o event_groups.o timers.o queue.o: CFLAGS += -D_ESP_FREERTOS_INTERNAL
COMPONENT_ADD_LDFRAGMENTS += linker.lf
