HTML_DOCS=fll
PDF_DOCS= fll.pdf

EXTRA_DIST = Doxyfile-html Doxyfile-pdf Doxyfile-common

if !FLL_BUILD_DOC
HTML_DOCSDIR = $(srcdir)/../generated/html/
PDF_DOCSDIR = $(srcdir)/../generated/pdf/
else
HTML_DOCSDIR = ./
PDF_DOCSDIR = ./

all-local: $(HTML_DOCS) $(PDF_DOCS)

fll: Doxyfile-html
	$(DOXYGEN) $(DOXYDOC_OPTS) $<

fll.pdf: FORCE
	$(DOXYGEN) $(DOXYDOC_OPTS) Doxyfile-pdf
	$(MAKE) -C fll-latex refman.pdf
	mv fll-latex/refman.pdf $@

distclean-local:
	for dir in $(HTML_DOCS) fll-latex; do \
	    if test -d $$dir ; then $(RM) -R $$dir ; fi ; \
	done

.PHONY: Doxyfile-html FORCE

.DELETE_ON_ERROR:

endif

include $(top_srcdir)/doc/install-dist.rules

install-data-local: install-docs-local

dist-hook: dist-docs-hook

uninstall-local: uninstall-docs
