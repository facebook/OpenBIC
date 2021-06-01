#ifndef __FUN_ID_H__
#define __FUN_ID_H__
#include "util.h"

/* Pin ID */
typedef enum {
    FUN_NONE = -1,
#define FUN_DEFINE(fun, ...) CONCAT(FUN_, fun),
    #include "fun_def_list.h"
#undef FUN_DEFINE
    MAX_FUN_ID,
} aspeed_fun_id_t;

#define GET_FUN_ID(fun) CONCAT(FUN_, fun)

#endif /* #ifndef __FUN_ID_H__ */
