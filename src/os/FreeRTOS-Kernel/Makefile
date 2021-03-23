-include $(KCONFIG_CONFIG)
obj-y += event_groups.o list.o queue.o stream_buffer.o tasks.o timers.o
obj-$(CONFIG_USE_CO_ROUTINES) += croutine.o
obj-y += portable/GCC/$(OS_PORT)/
obj-y += portable/MemMang/