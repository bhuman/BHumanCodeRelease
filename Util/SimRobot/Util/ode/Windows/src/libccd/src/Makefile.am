SUBDIRS = . testsuites

AM_CFLAGS = -std=c99

noinst_LTLIBRARIES = libccd.la

libccd_la_SOURCES = alloc.c ccd/alloc.h \
		ccd/compiler.h \
		ccd/dbg.h \
		ccd.c ccd/ccd.h \
		ccd/list.h \
		polytope.c ccd/polytope.h \
		ccd/quat.h \
		ccd/simplex.h \
		support.c ccd/support.h \
		vec3.c ccd/vec3.h \
        mpr.c

nodist_libccd_la_SOURCES = ccd/precision.h

EXTRA_DIST = ccd/precision.h.in
