noinst_PROGRAMS= dstest
AM_CPPFLAGS = -I$(top_srcdir)/drawstuff/src -I$(top_srcdir)/include

dstest_SOURCES= dstest.cpp
dstest_LDADD=$(top_builddir)/drawstuff/src/libdrawstuff.la \
            @GL_LIBS@

if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	$(WINDRES) $(top_srcdir)/drawstuff/src/resources.rc -o resources.o

dstest_LDADD += resources.o
endif

