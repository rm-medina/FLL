bin_PROGRAMS = fll test-display

test_display_SOURCES =	\
	test-display.c

test_display_CPPFLAGS = \
	$(fll_CPPFLAGS)

test_display_LDADD = \
	$(fll_LDADD)

fll_SOURCES = \
	main.c	

fll_CPPFLAGS =		\
	@FLL_CFLAGS@ @FLL_EXTRA_CFLAGS@	\
	-I$(top_srcdir)/include

fll_LDFLAGS = @FLL_LDFLAGS@

fll_LDADD =		\
	-lpthread -lrt

if USEOPENCV
if HAVE_OPENCV2
fll_CPPFLAGS +=  @opencvinc@ -DHAVE_OPENCV2 
fll_LDFLAGS = -L @opencvlib@
fll_SOURCES += capture.c
fll_LDADD += @OPENCV_ADD_LDFLAG@
endif
endif

