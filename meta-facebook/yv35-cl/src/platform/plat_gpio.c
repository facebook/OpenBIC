#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "pal.h"
#include "plat_gpio.h"
#include "plat_func.h"


#define gpio_name_to_num(x) #x,
const char * const gpio_name[] = { name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE name_gpioF name_gpioG name_gpioH name_gpioI name_gpioJ name_gpioK name_gpioL name_gpioM name_gpioN name_gpioO name_gpioP name_gpioQ name_gpioR name_gpioS name_gpioT name_gpioU };
#undef gpio_name_to_num


GPIO_CFG plat_gpio_cfg[] = {
//  chip,      number,   is_init, direction,    status,     property,    int_type,              int_cb
//  Defalut              DISABLE  GPIO_INPUT    LOW         PUSH_PULL    GPIO_INT_DISABLE       NULL
  { chip_gpio,  0,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO A group
  { chip_gpio,  1,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  2,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_EDGE_RISING,  ISR_slp3        },
//  { chip_gpio,  2,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  3,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  4,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  5,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  6,       ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  7,       ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  8,       ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO B group
//  { chip_gpio,  9,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_EDGE_RISING,  ISR_usbhub      }, 
  { chip_gpio,  9,       ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  10,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  11,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  12,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  13,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  14,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  15,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_BOTH,    ISR_DC_on       },
  { chip_gpio,  16,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO C group
  { chip_gpio,  17,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  18,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  19,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  20,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  21,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  22,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  23,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  24,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            }, // GPIO D group
  { chip_gpio,  25,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  26,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  27,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  28,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  29,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  30,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  31,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
//  { chip_gpio,  32,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_BOTH,    ISR_DC_on       }, // GPIO E group
  { chip_gpio,  32,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO E group
  { chip_gpio,  33,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  34,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  35,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  36,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  37,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  38,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
//  { chip_gpio,  39,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_RISING,  ISR_pltrst      },
  { chip_gpio,  39,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  40,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO F group
  { chip_gpio,  41,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  42,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  43,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  44,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  45,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  46,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  47,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  48,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO G group
  { chip_gpio,  49,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  50,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  51,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  52,      ENABLE,  GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  53,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  54,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  55,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  56,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO H group
  { chip_gpio,  57,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  58,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  59,      ENABLE,  GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  60,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  61,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  62,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  63,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  64,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO I group
  { chip_gpio,  65,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  66,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  67,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  68,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  69,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  70,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  71,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  72,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO J group
  { chip_gpio,  73,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  74,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  75,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  76,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  77,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  78,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  79,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  80,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO K group
  { chip_gpio,  81,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  82,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  83,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  84,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  85,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  86,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  87,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  88,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO L group
  { chip_gpio,  89,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  90,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  91,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  92,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  93,      ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  94,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  95,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  96,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO M group
  { chip_gpio,  97,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  98,      ENABLE,  GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  99,      DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  100,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  101,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  102,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  103,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  104,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO N group
  { chip_gpio,  105,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  106,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  107,     ENABLE,  GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  108,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  109,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  110,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  111,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  112,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO O group
  { chip_gpio,  113,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  114,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  115,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  116,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  117,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  118,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  119,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  120,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO P group
  { chip_gpio,  121,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  122,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  123,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  124,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  125,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  126,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  127,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  128,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO Q group
  { chip_gpio,  129,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  130,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  131,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  132,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  133,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  134,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  135,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  136,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO R group
  { chip_gpio,  137,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  138,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  139,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  140,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  141,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  142,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  143,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  144,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO S group
  { chip_gpio,  145,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  146,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  147,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  148,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  149,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  150,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  151,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  152,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO T group
  { chip_gpio,  153,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  154,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  155,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  156,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  157,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  158,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  159,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  160,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, // GPIO U group
  { chip_gpio,  161,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            }, 
  { chip_gpio,  162,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  163,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  164,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  165,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  166,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
  { chip_gpio,  167,     DISABLE, GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL            },
};

bool pal_load_gpio_config(void) {
  memcpy(&gpio_cfg[0], &plat_gpio_cfg[0], sizeof(plat_gpio_cfg));
  return 1;
};

