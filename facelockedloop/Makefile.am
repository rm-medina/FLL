bin_PROGRAMS = fll test-haar test-display test-BGR2GRAY

test_display_SOURCES =	\
	test-display.c

test_display_CPPFLAGS = \
	$(fll_CPPFLAGS)

test_display_LDADD = \
	$(fll_LDADD)

test_haar_SOURCES =	\
	test-haar.c

test_haar_CPPFLAGS = \
	$(fll_CPPFLAGS)

test_haar_LDADD = \
	$(fll_LDADD)

test_BGR2GRAY_SOURCES =	\
	test-haar.c

test_BGR2GRAY_CPPFLAGS = \
	$(fll_CPPFLAGS)

test_BGR2GRAY_LDADD = \
	$(fll_LDADD)

fll_SOURCES = \
	main.c	\
	pipeline.c \
	pipeline.h \
	capture.c \
	capture.h \
	detect.c \
	detect.h \
	track.c	\
	track.h \
	store.h

fll_CPPFLAGS =		\
	@FLL_CFLAGS@ @FLL_EXTRA_CFLAGS@	\
	-I$(top_srcdir)/include		\
	-I$(top_builddir)/servolib/lib


fll_LDFLAGS = @FLL_LDFLAGS@

fll_LDADD =		\
	../servolib/libservolib.la \
	-lpthread -lrt

if USEOPENCV
if HAVE_OPENCV2
fll_CPPFLAGS +=  @opencvinc@ -DHAVE_OPENCV2 
fll_LDFLAGS = -L @opencvlib@
fll_LDADD += @OPENCV_ADD_LDFLAG@
endif
endif

