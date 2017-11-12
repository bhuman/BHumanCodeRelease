SUBDIRS = joints UnitTest++

AM_CPPFLAGS = -I$(srcdir)/UnitTest++/src \
              -I$(top_srcdir)/include \
              -I$(top_builddir)/include \
              -I$(top_srcdir)/ode/src

if GIMPACT
    AM_CPPFLAGS += -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT
endif
if OPCODE
    AM_CPPFLAGS += -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE
endif


check_PROGRAMS = tests

TESTS = tests

tests_SOURCES = \
                collision.cpp \
                friction.cpp \
                joint.cpp \
                main.cpp \
                odemath.cpp

tests_LDADD = \
    $(top_builddir)/ode/src/libode.la \
    joints/*.o \
    UnitTest++/src/libunittestpp.la
