#include <stdio.h>
#include <stdlib.h>

#include "panic.h"

static void osmo_panic_default(const char *fmt, va_list args)
{
	vfprintf(stderr, fmt, args);
	abort();
}

void osmo_panic(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	osmo_panic_default(fmt, args);
	va_end(args);
}
 

void osmo_set_panic_handler(osmo_panic_handler_t h)
{
}

