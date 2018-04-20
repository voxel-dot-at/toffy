AC_INIT([uri_grammar], [0.3], [braden@endoframe.com])
AC_CONFIG_FILES([Makefile]
                [src/Makefile]
                [tests/Makefile]
                [tests/atlocal])
AC_CONFIG_TESTDIR([tests])
AC_PROG_CXX
AM_INIT_AUTOMAKE([foreign])

#
# Allow users to specify any Boost library name suffix
#
AS_IF([test -z "${BOOST_LIB_SUFFIX+x}"], [BOOST_LIB_SUFFIX=-mt])
AC_ARG_VAR([BOOST_LIB_SUFFIX], [Boost library name suffix [default=-mt]])

AC_CACHE_CHECK([for boost_thread$BOOST_LIB_SUFFIX library],
[ov_cv_boost_thread],
[ov_cv_boost_thread=no
ov_save_LIBS=$LIBS
LIBS="-lboost_thread$BOOST_LIB_SUFFIX -lboost_system$BOOST_LIB_SUFFIX $LIBS"
AC_LANG_PUSH([C++])
AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <boost/thread.hpp>]],
                                [[boost::thread t]])],
               [ov_cv_boost_thread=yes])
AC_LANG_POP
LIBS=$ov_save_LIBS
])
AS_IF([test X$ov_cv_boost_thread = Xno],
      [AC_MSG_FAILURE([libboost_thread$BOOST_LIB_SUFFIX not found])])

AC_OUTPUT
