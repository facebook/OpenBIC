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
//  chip,      number,   is_init, is_latch,  direction,    status,     property,    int_type,              int_cb
//  Defalut              DISABLE  DISABLE    GPIO_INPUT    LOW         PUSH_PULL    GPIO_INT_DISABLE       NULL
  { chip_gpio,  0,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO A group
  { chip_gpio,  1,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  2,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_BOTH,    ISR_PWROK_SLOT1   },
  { chip_gpio,  3,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  4,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  5,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  6,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  7,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  8,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO B group
  { chip_gpio,  9,       ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  10,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  11,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  12,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  13,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_BOTH,    ISR_PWROK_SLOT3   },
  { chip_gpio,  14,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  15,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  16,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO C group
  { chip_gpio,  17,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  18,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  19,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  20,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  21,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  22,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_EDGE_BOTH,    ISR_sled_cycle    },
  { chip_gpio,  23,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  24,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO D group
  { chip_gpio,  25,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  26,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  27,      ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  28,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  29,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  30,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  31,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  32,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO E group
  { chip_gpio,  33,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  34,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  35,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  36,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  37,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  38,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  39,      ENABLE,  DISABLE,   GPIO_INPUT,   GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  40,      DISABLE, DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              }, // GPIO F group
  { chip_gpio,  41,      DISABLE, DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  42,      DISABLE, DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  43,      DISABLE, DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  44,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  45,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  46,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  47,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  48,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO G group
  { chip_gpio,  49,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  50,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  51,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  52,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  53,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  54,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  55,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  56,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              }, // GPIO H group
  { chip_gpio,  57,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  58,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  59,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  60,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  61,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  62,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  63,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  64,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              }, // GPIO I group
  { chip_gpio,  65,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  66,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  67,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  68,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  69,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  70,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  71,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  72,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              }, // GPIO J group
  { chip_gpio,  73,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  74,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  75,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_HIGH,  OPEN_DRAIN,  GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  76,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  77,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  78,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  79,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  80,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO K group
  { chip_gpio,  81,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  82,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  83,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  84,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  85,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  86,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  87,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  88,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO L group
  { chip_gpio,  89,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  90,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  91,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  92,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  93,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  94,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  95,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  96,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO M group
  { chip_gpio,  97,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  98,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  99,      DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  100,     ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_HIGH,  PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  101,     ENABLE,  DISABLE,   GPIO_OUTPUT,  GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  102,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  103,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  104,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO N group
  { chip_gpio,  105,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  106,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  107,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  108,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  109,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  110,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  111,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  112,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO O group
  { chip_gpio,  113,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  114,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  115,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  116,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  117,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  118,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  119,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  120,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO P group
  { chip_gpio,  121,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  122,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  123,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  124,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  125,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  126,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  127,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  128,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO Q group
  { chip_gpio,  129,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  130,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  131,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  132,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  133,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  134,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  135,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  136,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO R group
  { chip_gpio,  137,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  138,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  139,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  140,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  141,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  142,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  143,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  144,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO S group
  { chip_gpio,  145,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  146,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  147,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  148,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  149,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  150,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  151,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  152,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO T group
  { chip_gpio,  153,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  154,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  155,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  156,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  157,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  158,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  159,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  160,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              }, // GPIO U group
  { chip_gpio,  161,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  162,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  163,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  164,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  165,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  166,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
  { chip_gpio,  167,     DISABLE, DISABLE,   GPIO_INPUT,   GPIO_LOW,   PUSH_PULL,   GPIO_INT_DISABLE,      NULL              },
};

bool pal_load_gpio_config(void) {
  memcpy(&gpio_cfg[0], &plat_gpio_cfg[0], sizeof(plat_gpio_cfg));
  return 1;
};

