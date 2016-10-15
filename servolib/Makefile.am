lib_LTLIBRARIES = libservolib.la

libservolib_la_SOURCES =      	\
	servoio.c

libservolib_la_CPPFLAGS = 	\
	@FLL_CFLAGS@          	\
	-I$(top_srcdir)/include

bin_PROGRAMS = test-servo

test_servo_SOURCES =		\
	test-servo.c

test_servo_CPPFLAGS =		\
	@FLL_CFLAGS@ 		\
	-I$(top_srcdir)/include

test_servo_LDADD = libservolib.la -lrt -lm

