/* 765: Library to emulate the uPD765a floppy controller (aka Intel 8272)

    Copyright (C) 2002  John Elliott <jce@seasip.demon.co.uk>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Library General Public License for more details.

    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#include <stdio.h>
#include <stdarg.h>
#include "765.h"

void fdc_dprintf(int debuglevel, const char *fmt, ...)
{
    va_list ap;
    if (debuglevel <= 1)
    {
        va_start( ap, fmt );
        dsk765PrintDebug(false, fmt, ap);
        va_end( ap );
    }
}
