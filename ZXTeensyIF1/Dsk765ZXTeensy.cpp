
#include "Dsk765ZXTeensy.h"

extern bool menuVaPrintDebug(bool clearDebug, const char *fmt, va_list ap);

extern "C" {
    bool dsk765PrintDebug(bool clearDebug, const char *fmt, va_list ap)
    {
        return menuVaPrintDebug(clearDebug, fmt, ap);
    }

    #include "lib765/765drive.c"
    #include "lib765/765dsk.c"
    #include "lib765/765fdc.c"
    #include "lib765/error.c"
}
