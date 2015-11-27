AM_CPPFLAGS = -I$(top_srcdir)/include \
        -I$(top_builddir)/include \
        -DDRAWSTUFF_TEXTURE_PATH="\"$(abs_top_srcdir)/drawstuff/textures\""

if X11
AM_LDFLAGS = $(X_PRE_LIBS) $(X_LIBS) $(X_EXTRA_LIBS)
endif

# On Windows, GL_LIBS must go after libdrawstuff.la.
LDADD = $(top_builddir)/drawstuff/src/libdrawstuff.la \
        $(top_builddir)/ode/src/libode.la @GL_LIBS@

noinst_HEADERS = basket_geom.h texturepath.h world_geom3.h bunny_geom.h convex_bunny_geom.h \
                 icosahedron_geom.h

AM_DEFAULT_SOURCE_EXT = .cpp

noinst_PROGRAMS = \
                demo_boxstack \
                demo_buggy \
                demo_cards \
                demo_chain1 \
                demo_chain2 \
                demo_collision \
                demo_crash \
                demo_cylvssphere \
                demo_dball \
                demo_dhinge \
                demo_transmission \
                demo_feedback \
                demo_friction \
                demo_gyroscopic \
                demo_gyro2 \
                demo_heightfield \
                demo_hinge \
                demo_I \
                demo_jointPR \
                demo_joints \
                demo_jointPU \
                demo_kinematic \
                demo_motion \
                demo_motor \
                demo_ode \
                demo_piston \
                demo_plane2d \
                demo_rfriction \
                demo_slider \
                demo_space \
                demo_space_stress \
                demo_step \
                demo_tracks

demo_chain1_SOURCES = demo_chain1.c
demo_chain1_LDADD = $(LDADD) -lstdc++ 


if TRIMESH
noinst_PROGRAMS += \
                demo_basket \
                demo_cyl \
                demo_moving_trimesh \
                demo_moving_convex \
                demo_trimesh

AM_CPPFLAGS += -DdTRIMESH_ENABLED
endif



if WIN32
resources.o: $(top_srcdir)/drawstuff/src/resources.rc $(top_srcdir)/drawstuff/src/resource.h
	@WINDRES@ $(top_srcdir)/drawstuff/src/resources.rc -o resources.o
LDADD += resources.o
endif
