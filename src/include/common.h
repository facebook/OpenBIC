#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "config.h"
#include "util.h"
#include "io.h"
#include "stdlib.h"
#include CONFIG_SOC_INCLUDE_FILE

#define	DEBUG_HALT()	{ volatile int halt = 1; while (halt); }