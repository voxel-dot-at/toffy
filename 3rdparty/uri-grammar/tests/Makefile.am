AM_CPPFLAGS = \
        -I$(top_srcdir)/src \
        -DBOOST_SPIRIT_THREADSAFE \
        -DBOOST_TEST_DYN_LINK

TESTS = unit-tests

TESTS_ENVIRONMENT = \
        BOOST_TEST_LOG_LEVEL=test_suite \
        BOOST_TEST_REPORT_LEVEL=detailed

check_PROGRAMS = unit-tests parse-uri

unit_tests_SOURCES = unit_tests.cpp
unit_tests_LDADD = \
        -lboost_unit_test_framework$(BOOST_LIB_SUFFIX) \
        -lboost_thread$(BOOST_LIB_SUFFIX) \
        -lboost_system$(BOOST_LIB_SUFFIX)

parse_uri_SOURCES = parse_uri.cpp
parse_uri_LDADD = \
        -lboost_program_options$(BOOST_LIB_SUFFIX) \
        -lboost_thread$(BOOST_LIB_SUFFIX) \
        -lboost_system$(BOOST_LIB_SUFFIX)

EXTRA_DIST = \
        package.m4 \
        testsuite.at \
        testsuite

TESTSUITE = $(srcdir)/testsuite
check-local: atconfig atlocal $(TESTSUITE)
	$(SHELL) $(TESTSUITE)

AUTOTEST = autom4te --language=autotest
$(TESTSUITE): $(srcdir)/package.m4 $(srcdir)/testsuite.at
	$(AUTOTEST) -I $(srcdir) $@.at -o $@.tmp; mv $@.tmp $@

$(srcdir)/package.m4: $(top_srcdir)/configure.ac
	{                                      \
          echo '# Signature of the current package.'; \
          echo 'm4_define([AT_PACKAGE_NAME],      [@PACKAGE_NAME@])'; \
          echo 'm4_define([AT_PACKAGE_TARNAME],   [@PACKAGE_TARNAME@])'; \
          echo 'm4_define([AT_PACKAGE_VERSION],   [@PACKAGE_VERSION@])'; \
          echo 'm4_define([AT_PACKAGE_STRING],    [@PACKAGE_STRING@])'; \
          echo 'm4_define([AT_PACKAGE_BUGREPORT], [@PACKAGE_BUGREPORT@])'; \
        } > $(srcdir)/package.m4

clean-local:
	test ! -f $(TESTSUITE) || $(SHELL) $(TESTSUITE) --clean
	rm -f *.tmp
	rm -f -r autom4te.cache

DISTCLEANFILES = atconfig
