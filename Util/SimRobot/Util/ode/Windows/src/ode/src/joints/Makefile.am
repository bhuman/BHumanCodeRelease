AM_CPPFLAGS = -I$(top_srcdir)/include \
        -I$(top_builddir)/include \
        -I$(top_srcdir)/ode/src \
        -D__ODE__


if ENABLE_OU

AM_CPPFLAGS += -I$(top_srcdir)/ou/include


endif


noinst_LTLIBRARIES = libjoints.la

libjoints_la_SOURCES =  joints.h \
                        joint.h joint.cpp \
                        joint_internal.h \
                        ball.h ball.cpp \
                        dball.h dball.cpp \
                        dhinge.h dhinge.cpp \
                        transmission.h transmission.cpp \
                        hinge.h hinge.cpp \
                        slider.h slider.cpp \
                        contact.h contact.cpp \
                        universal.h universal.cpp \
                        hinge2.h hinge2.cpp \
                        fixed.h fixed.cpp \
                        null.h null.cpp \
                        amotor.h amotor.cpp \
                        lmotor.h lmotor.cpp \
                        plane2d.h plane2d.cpp \
                        pu.h pu.cpp \
                        pr.h pr.cpp \
                        piston.h piston.cpp

