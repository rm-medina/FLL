#ifndef __DEBUG_H_
#define __DEBUG_H_

#include <stdio.h>
#include <stdarg.h>

void __debug(const char *who, const char *fmt, ...);

#define debug(__obj, __fmt, args...)					\
	do {								\
		const char *__who = NULL;				\
		if (__debug_enabled) {					\
			if (__obj)					\
				__who = (__obj)->params.name;		\
			__debug(__who, __fmt, ##args);			\
		}							\
	} while (0)


extern int __debug_enabled;

static inline void enable_debug(void)
{
	__debug_enabled = 1;
}

#endif
