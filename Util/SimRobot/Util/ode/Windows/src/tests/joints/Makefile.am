AM_CPPFLAGS = -I$(srcdir)/../UnitTest++/src \
              -I$(top_srcdir)/include \
              -I$(top_builddir)/include \
              -I$(top_srcdir)/ode/src

check_LTLIBRARIES = libjoints.la

libjoints_la_LDFLAGS = -static

libjoints_la_SOURCES = \
    amotor.cpp \
    ball.cpp \
    dball.cpp \
    fixed.cpp \
    hinge.cpp \
    hinge2.cpp \
    piston.cpp \
    pr.cpp \
    pu.cpp \
    slider.cpp \
    universal.cpp
