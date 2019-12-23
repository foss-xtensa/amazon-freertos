#include <stddef.h>
#include <stdio.h>
#include "../../../tasks.c"

#define DEFINE(sym,val) \
    printf("#define %s %d /* %s */\n", #sym, (val), #val)

int main()
{
    DEFINE(TCB_TOP_OF_STACK_OFF, offsetof(TCB_t, pxTopOfStack));
#if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
    DEFINE(TCB_END_OF_STACK_OFF, offsetof(TCB_t, pxEndOfStack));
#endif
#if ( configUSE_NEWLIB_REENTRANT == 1 )
    DEFINE(TCB_IMPURE_PTR_OFF, offsetof(TCB_t, xNewLib_reent));
#endif
}
