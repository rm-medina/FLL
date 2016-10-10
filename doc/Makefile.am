SUBDIRS=doxygen

GENERATE_DOC=generate-doc

generate-doc: FORCE
	@if test \! -e $(GENERATE_DOC) ; then \
		echo "$@ missing generate-doc." ; \
	else \
		srcdir=$(srcdir) $(GENERATE_DOC) ; \
	fi

.PHONY: FORCE
