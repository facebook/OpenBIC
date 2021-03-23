#include "pinctrl_aspeed.h"
#define PIN_DEFINE(pin) \
    [pin] = -1,
int aspeed_pin_desc_table[] __attribute__((section(".pinctrl_pin_desc"))) = 
{
    #include "pin_def_list.h"
};
#undef PIN_DEFINE

#ifdef CONFIG_PINCTRL_NAME_TABLE

#define PIN_DEFINE(pin) \
    [pin] = # pin,
const char *aspeed_pin_name[] __attribute__((section(".pinctrl_pin_name"))) =
{
    #include "pin_def_list.h"
};
#undef PIN_DEFINE

#define FUN_DEFINE(fun, ...) \
    [GET_FUN_ID(fun)] = # fun,
const char *aspeed_fun_name[] __attribute__((section(".pinctrl_fun_name"))) =
{
    #include "fun_def_list.h"
};
#undef FUN_DEFINE

#define SIG_DEFINE(sig, ...) \
    [sig] = # sig,
const char *aspeed_sig_name[] __attribute__((section(".pinctrl_sig_name"))) =
{
    #include "sig_def_list.h"
};
#undef SIG_DEFINE

#endif /* #ifdef CONFIG_PINCTRL_NAME_TABLE */