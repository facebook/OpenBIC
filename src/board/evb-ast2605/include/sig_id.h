#ifndef __SIG_ID_H__
#define __SIG_ID_H__

/* Pin ID */
typedef enum {
    SIG_NONE = -1,
    SIG_GPIO = 0,
#define SIG_DEFINE(sig, ...) sig,
    #include "sig_def_list.h"
#undef SIG_DEFINE
    MAX_SIG_ID,
} aspeed_sig_id_t;

#endif /* #ifndef __SIG_ID_H__ */
