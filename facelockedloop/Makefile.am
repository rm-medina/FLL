bin_PROGRAMS = fll

fll_SOURCES = \
	main.c

fll_CPPFLAGS =		\
	@FLL_CFLAGS@	\
	-I$(top_srcdir)/include

fll_LDFLAGS = @FLL_LDFLAGS@

fll_LDADD =		\
	-lpthread -lrt
