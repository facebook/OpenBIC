#ifndef __PIN_ID_H__
#define __PIN_ID_H__

/* Pin ID */
typedef enum {
    PIN_NONE = -1,
#define PIN_DEFINE(pin) pin,
    #include "pin_def_list.h"
#undef PIN_DEFINE
    MAX_PIN_ID,
} aspeed_pin_id_t;

#endif /* #ifndef __PIN_ID_H__ */
