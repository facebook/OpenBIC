#include "pinctrl_aspeed.h"

#define SIG_DEFINE(sig, pin, ...) SIG_DECL(sig, pin, __VA_ARGS__);
#include "sig_def_list.h"
#undef SIG_DEFINE

#define FUN_DEFINE(fun, ... ) FUN_DECL(fun, __VA_ARGS__);
#include "fun_def_list.h"
#undef FUN_DEFINE