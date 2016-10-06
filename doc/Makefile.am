SUBDIRS=doxygen

GENERATE_DOC=$(top_srcdir)/scripts/generate-doc

generate-doc: FORCE
	@if test \! -e $(GENERATE_DOC) -o \! -e $(top_srcdir)/.git ; then \
		echo "$@ wants $(top_srcdir) to be a GIT working tree." ; \
	else \
		srcdir=$(srcdir) $(GENERATE_DOC) ; \
	fi

.PHONY: FORCE
