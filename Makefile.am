# FLL project

SUBDIRS = include servolib facelockedloop @FLL_MAYBE_DOCDIR@

SUBDIRS += @FLL_MAYBE_DOCDIR@

servolib-local:
	@make -C servolib $@

fll-local:
	@make -C facelockedloop $@

all clean install:
	@make $@

dist-hook:
	rm -fr `find $(distdir) -name '.#*' -o -name '*~' -o -name autom4te.cache`
	test -e $(srcdir)/doc || rm -rf $(distdir)/doc

.PHONY: all clean install
