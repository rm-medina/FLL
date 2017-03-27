/**
 * @file facelockedloop/debug.c
 * @brief debug filter
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <pthread.h>
#include <stdarg.h>
#include <unistd.h>
#include <execinfo.h>
#include <malloc.h>
#include <stdlib.h>
#include <pthread.h>
#include "debug.h"

static pthread_mutex_t output_lock = PTHREAD_MUTEX_INITIALIZER;

int __debug_enabled;

void __debug(const char *who, const char *fmt, ...)
{
	char *header, *msg;
	FILE *fp = stderr;
	int hlen, mlen;
	va_list ap;

	va_start(ap, fmt);

	hlen = asprintf(&header, "[%s] ", who ?: "...");
	mlen = vasprintf(&msg, fmt, ap);

	pthread_mutex_lock(&output_lock);
	write(fileno(fp), header, hlen);
	write(fileno(fp), msg, mlen);
	write(fileno(fp), "\n", 1);
	pthread_mutex_unlock(&output_lock);

	free(header);
	free(msg);

	va_end(ap);
}
