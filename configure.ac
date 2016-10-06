dnl Process this file with autoconf to produce a configure script.
AC_CONFIG_MACRO_DIR([m4])

AC_INIT([FLL],m4_normalize(m4_include([config/version])),raquel.medina.rodriguez@gmail.com)

FLL_VERSION_MAJOR=`expr $PACKAGE_VERSION : '\([[0-9]]*\'`
FLL_VERSION_MINOR=`expr $PACKAGE_VERSION : '\[[0-9]]*\.\([[0-9]]*\)'`
FLL_REVISION=`expr $PACKAGE_VERSION : '\([[0-9]]*\.([[0-9]]*\.\([[0-9]]*\)'`
test "x$FLL_REVISION" = "x" && FLL_REVISION=0

AC_CONFIG_HEADERS(include/fll_config.h)
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR(facelockedloop/main.c)
AC_PREFIX_DEFAULT(/usr/local/fll)
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_PROG_INSTALL
AM_PROG_LEX
AC_PROG_YACC

#Do no let autoconf set the default value of CFLAGS
if $FLL_EMPTY_CFLAGS; then
        CFLAGS=""
fi

C_ARG_WITH(cc,
    AS_HELP_STRING([--with-cc=compiler],[use specific C compiler]),
    [
	case "$withval" in
	"" | y | ye | yes | n | no)
	    AC_MSG_ERROR([You must supply an argument to --with-cc.])
	  ;;
	esac
	CC="$withval"
    ])
AC_PROG_CC


if test "x$CFLAGS" = "x"; then
	FLL_EMPTY_CFLAGS=true
else
	FLL_EMPTY_CFLAGS=false
fi

AC_DEFINE_UNQUOTED(FLL_BUILD_STRING,"$build",[Build system alias])
FLL_BUILD_STRING="$build"
AC_DEFINE_UNQUOTED(FLL_HOST_STRING,"$host",[Host system alias])
FLL_HOST_STRING="$host"
AC_DEFINE_UNQUOTED(FLL_COMPILER,"`$CC -v 2>&1 | tail -n 1`",[Compiler])

AM_INIT_AUTOMAKE([foreign no-exeext dist-bzip2])
AM_MAINTAINER_MODE
AC_PROG_LIBTOOL

if test x$host_alias = x; then
  build_for=$host
else
  build_for=$host_alias
fi

AC_MSG_CHECKING([for target architecture])
case "$build_for" in
 arm*-*)
	target_arch=arm
	;;
 x86_64-*|amd64-*)
	target_arch=x86_64
	;;
 i*86*-*)
	target_arch=x86_32
	;;
 *) echo ""
   echo "*******************************************"
   echo "Unsupported target: $build_for -- Sorry."
   echo "*******************************************"
   echo ""
   exit 1;;
esac
AC_MSG_RESULT([$target_arch])

dnl Debug build (default: off)

debug_mode=
AC_MSG_CHECKING(whether to enable debug mode)
AC_ARG_ENABLE(debug,
	AS_HELP_STRING([--enable-debug[=full]], [Enable debug mode in programs]),
	[case "$enableval" in
	y | yes) debug_mode=partial ;;
	full)    debug_mode=full ;;
	*) unset debug_mode ;;
	esac])
AC_MSG_RESULT(${debug_mode:-no})
AM_CONDITIONAL(FLL_BUILD_DEBUG,[test \! x$debug_mode = x])

dnl Stats build (default: off)

stats_mode=
AC_MSG_CHECKING(whether to enable stats mode)
AC_ARG_ENABLE(stats,
	AS_HELP_STRING([--enable-stats[=full]], [Enable stats mode in programs]),
	[case "$enableval" in
	y | yes) stats_mode=partial ;;
	full)    stats_mode=full ;;
	*) unset stats_mode ;;
	esac])
AC_MSG_RESULT(${stats_mode:-no})
AM_CONDITIONAL(FLL_BUILD_STATS,[test \! x$stats_mode = x])

dnl Host mode build (default: off)

host_mode=
AC_MSG_CHECKING(whether to enable host mode)
AC_ARG_ENABLE(host,
	AS_HELP_STRING([--enable-host], [Enable host mode]),
	[case "$enableval" in
	y | yes) host_mode=yes ;;
	*) unset host_mode ;;
	esac])
AC_MSG_RESULT(${host_mode:-no})
AM_CONDITIONAL(FLL_BUILD_HOST,[test \! x$host_mode = x])

checkflags="-nostdinc -isystem \$(SYSROOT)/usr/include -Wbitwise -Wno-transparent-union -D_GNU_SOURCE -D_XOPEN_SOURCE=500 -D_REENTRANT \$(DEFS) \$(DEFAULT_INCLUDES) \$(INCLUDES) \$(AM_CPPFLAGS) \$(CPPFLAGS) -I\$(top_srcdir)/include -isystem \$(shell \$(CC) -print-file-name=include) -include \$(top_builddir)/include/bighill_config.h \$(shell \$(CC) -dM -E -xc /dev/null|sed -e 's/^\\#define /-D/' -e \"s/ /=\'/\" -e \"s/\$\$/\'/\")"

dnl Used with sparse
AC_SUBST(CHECKFLAGS, $checkflags)

dnl Enable assertions (default: on)

use_assert=
if test -n "$debug_mode"; then
  use_assert=y
fi
AC_MSG_CHECKING(whether assertions should be enabled)
AC_ARG_ENABLE(assert,
	AS_HELP_STRING([--enable-assert], [Enable runtime assertions]),
	[case "$enableval" in
	y | yes) use_assert=y ;;
	n | no) unset use_assert ;;
	esac])
AC_MSG_RESULT(${use_assert:-no})

dnl Documentation package.

if test x"$docdir" = x; then
	docdir=$datadir/doc/$PACKAGE-$VERSION
fi
AC_SUBST(docdir)

if test \! -d $srcdir/doc ; then
    unset FLL_MAYBE_DOCDIR
else
    FLL_MAYBE_DOCDIR=doc
fi
AC_SUBST(FLL_MAYBE_DOCDIR)

AC_MSG_CHECKING(for Doxygen documentation)
AC_ARG_ENABLE(doc,
	AS_HELP_STRING([--enable-doc], [Build Doxygen documentation]),
	[case "$enableval" in
	y | yes) FLL_BUILD_DOC=y ;;
	*) unset FLL_BUILD_DOC ;;
	esac])

if test \! -d $srcdir/doc ; then
    if test x$FLL_BUILD_DOC = xy ;
    then
	AC_MSG_ERROR([documentation tree is missing.])
    fi
    AC_MSG_RESULT([not present])
else
    AC_MSG_RESULT(${FLL_BUILD_DOC:-no})
fi

AC_CHECK_PROG(DOXYGEN, doxygen, doxygen)

if test x${FLL_BUILD_DOC} = xy -a x"$DOXYGEN" = x ; then
   AC_MSG_ERROR([Missing the Doxygen tools to generate the documentation.])
fi

AC_CHECK_PROG(DOXYGEN_HAVE_DOT, dot, YES, NO)
if test x"$DOXYGEN_HAVE_DOT" = xYES ; then
   DOXYGEN_SHOW_INCLUDE_FILES=NO
else
   DOXYGEN_SHOW_INCLUDE_FILES=YES
fi

case "$FLL_DOC_DBX" in
y | ye | yes)
    DBX_DOC=yes ;;
*)
    DBX_DOC="" ;;
esac
DBX_NET="$FLL_DBX_NET"
DBX_ROOT="$FLL_DBX_ROOT"
DBX_XSL_ROOT="$FLL_DBX_XSL_ROOT"
DBX_DOC_INIT(doc/docbook, doc/generated, 4.2)

AC_MSG_CHECKING(whether using LaTeX non-stop mode)
AC_ARG_ENABLE(verbose-latex,
	AS_HELP_STRING([--enable-verbose-latex], [Uses LaTeX non-stop mode]),
	[case "$enableval" in
	y | yes) FLL_DOC_LATEX_NONSTOP=y ;;
	*) unset FLL_DOC_LATEX_NONSTOP ;;
	esac])
AC_MSG_RESULT(${FLL_DOC_LATEX_NONSTOP:-no})

if test x${FLL_DOC_LATEX_NONSTOP} = xy; then
   LATEX_BATCHMODE=NO
   LATEX_MODE=nonstopmode
else
   LATEX_BATCHMODE=YES
   LATEX_MODE=batchmode
fi

AC_DEFINE_UNQUOTED(FLL_VERSION_MAJOR,$BIGHILL_VERSION_MAJOR,[config])
AC_DEFINE_UNQUOTED(FLL_VERSION_MINOR,$BIGHILL_VERSION_MINOR,[config])
AC_DEFINE_UNQUOTED(FLL_REVISION_LEVEL,$BIGHILL_REVISION_LEVEL,[config])
AC_DEFINE_UNQUOTED(FLL_PREFIX,"$prefix",[config])

AH_VERBATIM(FLL___,[#ifdef __IN_FLL__])
AH_BOTTOM([#endif /* __IN_FLL__ */])

dnl Documentation.
AM_CONDITIONAL(FLL_BUILD_DOC,[test "$FLL_BUILD_DOC" = y])

dnl Common CFLAGS and LDFLAGS
FLL_CFLAGS="-D_GNU_SOURCE -D_REENTRANT -Wall -pipe -Winline $target_cflags"
FLL_LDFLAGS=-Wl,--export-dynamic

if test x$debug_mode = xpartial; then
   FLL_CFLAGS="-g -O2 $FLL_CFLAGS"
elif test x$debug_mode = xfull; then
   FLL_CFLAGS="-g -O0 $FLL_CFLAGS"
else
   FLL_CFLAGS="-O2 $FLL_CFLAGS"
fi
FLL_CFLAGS="$FLL_CFLAGS -D__IN_FLL__"

if test -n "$host_mode"; then
   AC_DEFINE(FLL_BUILD_HOST, 1, [config])
fi

if test -n "$debug_mode"; then
   AC_DEFINE(FLL_BUILD_DEBUG, 1, [config])
fi

if test -n "$stats_mode"; then
   AC_DEFINE(FLL_BUILD_STATS, 1, [config])
fi

if test x$use_assert = x; then
   FLL_CFLAGS="-DNDEBUG $FLL_CFLAGS"
fi

FLL_CFLAGS="$FLL_CFLAGS -Wextra -Wstrict-prototypes -Wmissing-prototypes -Wno-long-long -Wno-unused-parameter -Werror -fstrict-aliasing"
LIBS=

AC_MSG_CHECKING([for test source generation])
AC_RUN_IFELSE([AC_LANG_PROGRAM([[ ]], [[ ]])],
  [AC_MSG_RESULT(ok)], [AC_MSG_RESULT(failed)], [AC_MSG_RESULT(untestable)])
dnl
dnl Build the Makefiles
dnl

AC_SUBST(DOXYGEN_SHOW_INCLUDE_FILES)
AC_SUBST(DOXYGEN_HAVE_DOT)
AC_SUBST(DOXYGEN)
AC_SUBST(LATEX_BATCHMODE)
AC_SUBST(LATEX_MODE)

AC_SUBST(FLL_BUILD_STRING)
AC_SUBST(FLL_HOST_STRING)
AC_SUBST(FLL_CFLAGS)
AC_SUBST(FLL_LDFLAGS)

AC_CONFIG_FILES([ \
	Makefile \
	include/Makefile \
	include/midway/Makefile \
	servolib/Makefile \
	facelockedloop/Makefile \
	])

if test \! x$FLL_MAYBE_DOCDIR = x ; then
      AC_CONFIG_FILES([ \
	doc/Makefile \
	doc/doxygen/Makefile \
	doc/doxygen/Doxyfile-common \
	doc/doxygen/Doxyfile-html \
	doc/doxygen/Doxyfile-pdf \
	doc/docbook/Makefile \
	doc/docbook/catalog \
	doc/docbook/custom-stylesheets/Makefile \
	doc/docbook/custom-stylesheets/xsl/Makefile \
	doc/docbook/custom-stylesheets/xsl/common/Makefile \
	doc/docbook/custom-stylesheets/xsl/fo/Makefile \
	doc/docbook/custom-stylesheets/xsl/html/Makefile \
	doc/docbook/custom-stylesheets/xsl/html/chunk.xsl \
	doc/docbook/custom-stylesheets/xsl/html/onechunk.xsl \
	doc/docbook/fll/Makefile \
      ])
fi

AC_OUTPUT()
