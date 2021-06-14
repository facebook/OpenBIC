#include <stddef.h>
#include "FreeRTOS.h"

void* __wrap_malloc(size_t size)
{
	return pvPortMalloc(size);
}

void __wrap_free(void *memblock)
{
	vPortFree(memblock);
}