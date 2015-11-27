SUBDIRS = joints

AM_CPPFLAGS = -I$(top_srcdir)/include \
        -I$(top_builddir)/include \
        -D__ODE__

# convenience library to simulate per object cflags
noinst_LTLIBRARIES = libfast.la
libfast_la_SOURCES = fastldlt.c fastltsolve.c fastdot.c fastlsolve.c



lib_LTLIBRARIES = libode.la

libode_la_LDFLAGS = @EXTRA_LIBTOOL_LDFLAGS@ @ODE_VERSION_INFO@
libode_la_LIBADD = libfast.la joints/libjoints.la


# please, let's keep the filenames sorted
libode_la_SOURCES =     nextafterf.c \
                        array.cpp array.h \
                        box.cpp \
                        capsule.cpp \
                        collision_cylinder_box.cpp \
                        collision_cylinder_plane.cpp \
                        collision_cylinder_sphere.cpp \
                        collision_kernel.cpp collision_kernel.h \
                        collision_quadtreespace.cpp \
                        collision_sapspace.cpp \
                        collision_space.cpp \
                        collision_space_internal.h \
                        collision_std.h \
                        collision_transform.cpp collision_transform.h \
                        collision_trimesh_colliders.h \
                        collision_trimesh_disabled.cpp \
                        collision_trimesh_internal.h \
                        collision_util.cpp collision_util.h \
                        convex.cpp \
                        cylinder.cpp \
                        error.cpp error.h \
                        export-dif.cpp \
                        heightfield.cpp heightfield.h \
                        lcp.cpp lcp.h \
                        mass.cpp \
                        mat.cpp mat.h \
                        matrix.cpp matrix.h \
                        memory.cpp \
                        misc.cpp \
                        objects.cpp objects.h \
                        obstack.cpp obstack.h \
                        ode.cpp \
                        odeinit.cpp \
                        odemath.cpp odemath.h \
                        odeou.h \
                        odetls.h \
                        plane.cpp \
                        quickstep.cpp quickstep.h \
                        ray.cpp \
                        rotation.cpp \
                        sphere.cpp \
                        step.cpp step.h \
                        timer.cpp \
                        threading_atomics_provs.h \
                        threading_base.cpp threading_base.h \
                        threading_fake_sync.h \
                        threading_impl.cpp threading_impl.h \
                        threading_impl_posix.h \
                        threading_impl_templates.h \
                        threading_impl_win.h \
                        threading_pool_posix.cpp \
                        threading_pool_win.cpp \
                        threadingutils.h \
                        typedefs.h \
                        util.cpp util.h


###################################
#       O U    S T U F F
###################################


if ENABLE_OU

AM_CPPFLAGS += -I$(top_srcdir)/ou/include
libode_la_LIBADD += $(top_builddir)/ou/src/ou/libou.la
libode_la_SOURCES +=    odetls.cpp odetls.h \
                        odeou.cpp odeou.h

endif


###################################
#   G I M P A C T    S T U F F
###################################


if GIMPACT
AM_CPPFLAGS += -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT -I$(top_srcdir)/GIMPACT/include
libode_la_SOURCES+= collision_trimesh_gimpact.cpp

libode_la_LIBADD += $(top_builddir)/GIMPACT/src/libGIMPACT.la
libode_la_SOURCES +=    collision_trimesh_trimesh.cpp \
                        collision_trimesh_sphere.cpp \
                        collision_trimesh_ray.cpp \
                        collision_trimesh_opcode.cpp \
                        collision_trimesh_box.cpp \
                        collision_trimesh_ccylinder.cpp \
                        collision_trimesh_distance.cpp \
                        collision_trimesh_internal.h \
                        collision_cylinder_trimesh.cpp \
                        collision_trimesh_plane.cpp
endif



#################################
#   O P C O D E    S T U F F
#################################


if OPCODE
AM_CPPFLAGS += -I$(top_srcdir)/OPCODE -I$(top_srcdir)/OPCODE/Ice -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE
libode_la_LIBADD += $(top_builddir)/OPCODE/libOPCODE.la \
                    $(top_builddir)/OPCODE/Ice/libIce.la

libode_la_SOURCES+=     collision_trimesh_trimesh.cpp \
                        collision_trimesh_trimesh_new.cpp \
                        collision_trimesh_sphere.cpp \
                        collision_trimesh_ray.cpp \
                        collision_trimesh_opcode.cpp \
                        collision_trimesh_box.cpp \
                        collision_trimesh_ccylinder.cpp \
                        collision_trimesh_distance.cpp \
                        collision_trimesh_internal.h \
                        collision_cylinder_trimesh.cpp \
                        collision_trimesh_plane.cpp
endif


if LIBCCD
AM_CPPFLAGS += -I$(top_srcdir)/libccd/src -I$(top_builddir)/libccd/src -DdLIBCCD_ENABLED
libode_la_LIBADD += $(top_builddir)/libccd/src/libccd.la

libode_la_SOURCES += collision_libccd.cpp collision_libccd.h

if LIBCCD_BOX_CYL
AM_CPPFLAGS += -DdLIBCCD_BOX_CYL
endif

if LIBCCD_CYL_CYL
AM_CPPFLAGS += -DdLIBCCD_CYL_CYL
endif

if LIBCCD_CAP_CYL
AM_CPPFLAGS += -DdLIBCCD_CAP_CYL
endif

if LIBCCD_CONVEX_BOX
AM_CPPFLAGS += -DdLIBCCD_CONVEX_BOX
endif
if LIBCCD_CONVEX_CAP
AM_CPPFLAGS += -DdLIBCCD_CONVEX_CAP
endif
if LIBCCD_CONVEX_CYL
AM_CPPFLAGS += -DdLIBCCD_CONVEX_CYL
endif
if LIBCCD_CONVEX_SPHERE
AM_CPPFLAGS += -DdLIBCCD_CONVEX_SPHERE
endif
if LIBCCD_CONVEX_CONVEX
AM_CPPFLAGS += -DdLIBCCD_CONVEX_CONVEX
endif


endif
