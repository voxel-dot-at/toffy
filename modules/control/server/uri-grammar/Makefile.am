SUBDIRS = src tests

dist-hook:
	if test -d $(top_srcdir)/.svn; then \
    (cd $(top_srcdir); svn2cl --strip-prefix=trunk --group-by-day); \
    cp $(top_srcdir)/ChangeLog $(distdir); \
fi
