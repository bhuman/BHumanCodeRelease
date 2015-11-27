dnl AC_INIT does not take a macro as a version nr: set it separately! - Bram
AC_INIT([ODE],[0.13.1],[ode@ode.org])
ODE_VERSION=0.13.1
AC_SUBST(ODE_VERSION)

# Those are instructions from the Libtool manual:
#  1. Start with version information of `0:0:0' for each libtool library.
#
#  2. Update the version information only immediately before a public
#     release of your software.  More frequent updates are unnecessary,
#     and only guarantee that the current interface number gets larger
#     faster.
#
#  3. If the library source code has changed at all since the last
#     update, then increment REVISION (`C:R:A' becomes `C:r+1:A').
#
#  4. If any interfaces have been added, removed, or changed since the
#     last update, increment CURRENT, and set REVISION to 0.
#
#  5. If any interfaces have been added since the last public release,
#     then increment AGE.
#
#  6. If any interfaces have been removed since the last public release,
#     then set AGE to 0.
CURRENT=5
REVISION=0
AGE=1

AC_ARG_ENABLE(version-info,
        AS_HELP_STRING([--disable-version-info],
                [don't encode version information in the generated library]),
        version_info=$enableval,
        version_info=yes)
if test x$version_info = xyes
then
    ODE_VERSION_INFO="-version-info $CURRENT:$REVISION:$AGE"
else
    ODE_VERSION_INFO="-avoid-version"
fi
AC_SUBST(ODE_VERSION_INFO)


AC_CONFIG_SRCDIR([ode/src/ode.cpp])
AC_CONFIG_MACRO_DIR([m4])

AC_CANONICAL_HOST

AM_INIT_AUTOMAKE([1.10 foreign])
AC_CONFIG_HEADERS([ode/src/config.h])

dnl This is needed because we have subdirectories
AC_PROG_MAKE_SET
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CPP
AC_PROG_AWK
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MKDIR_P
LT_INIT([disable-shared win32-dll])
AC_CHECK_TOOLS([WINDRES], [windres])

AC_C_BIGENDIAN
AC_C_INLINE
AC_C_VOLATILE
PKG_PROG_PKG_CONFIG


dnl this may NOT be the machine on which the code is going to run in,
dnl so allow users to compile programs for their target machine.
pentium=no
cpu64=no
case "$host_cpu" in
  i586 | i686 | i786 )
        pentium=yes
        AC_DEFINE(PENTIUM,1,[compiling for a pentium on a gcc-based platform?])
    ;;
  x86_64* )
        pentium=yes
        cpu64=yes
        AC_DEFINE(X86_64_SYSTEM,1,[compiling for a X86_64 system on a gcc-based platform?])
    ;;
esac

AM_CONDITIONAL(X86_64_SYSTEM, test x$cpu64 = xyes)






dnl check for required headers
AC_CHECK_HEADERS( [alloca.h stdio.h inttypes.h stdint.h stdlib.h math.h \
                  string.h stdarg.h malloc.h float.h time.h sys/time.h \
                  limits.h stddef.h])


opcode=no
gimpact=no
AC_ARG_WITH(trimesh, AS_HELP_STRING([--with-trimesh=[opcode|gimpact|none]],
        [use the specified system for trimesh support @<:@default=opcode@:>@]),
        trimesh=$withval,trimesh=opcode
)
if test "$trimesh" = opcode
then
  opcode=yes
fi
if test "$trimesh" = gimpact
then
  gimpact=yes
fi

AM_CONDITIONAL(OPCODE,  test $opcode  = yes)
AM_CONDITIONAL(GIMPACT, test $gimpact = yes)
AM_CONDITIONAL(TRIMESH, test $opcode = yes -o $gimpact = yes)


AC_MSG_CHECKING(if double precision is requested)
AC_ARG_ENABLE(double-precision,
        AS_HELP_STRING([--enable-double-precision],
        [Configure ODE to work with double precision, if not specified, single precision is used @<:@default=no@:>@]),
        usedouble=$enableval,usedouble=no)
AC_MSG_RESULT([$usedouble])
if test "$usedouble" = yes;
then
        ODE_PRECISION=dDOUBLE
else
        ODE_PRECISION=dSINGLE
fi
AC_SUBST(ODE_PRECISION)


AC_ARG_WITH([drawstuff],
    AS_HELP_STRING([--with-drawstuff=X11|Win32|OSX|none],
        [force a particular drawstuff implementation or disable it.[default=autodetect]]),
    [drawstuff=$withval],[drawstuff=])

dnl Set some Platform Specific Variables
EXTRA_LIBTOOL_LDFLAGS=
case "$host_os" in
  cygwin* | mingw*)
    if test "x$drawstuff" = x
    then
       drawstuff="Win32" # if in a Windows enviroment
    fi
    EXTRA_LIBTOOL_LDFLAGS="-no-undefined"
    ;;
  *apple* | *darwin*) # For Mac OS X
    if test "x$drawstuff" = x
    then
       drawstuff="OSX"
    fi
    dnl We need to use C++ compilation and linking for ode on Mac
    dnl Might as well do it for all code.
    CC="$CXX"
    LINK="$CXXLINK"
    ;;
  *)
    if test "x$drawstuff" = x
    then
       drawstuff="X11" # if anything else default to X11
    fi
    ;;
esac
AC_SUBST(EXTRA_LIBTOOL_LDFLAGS)


dnl Set Drawstuff variables
AC_MSG_CHECKING([which drawstuff lib to build])
AC_MSG_RESULT($drawstuff)

if test "x$drawstuff" = "xX11"
then
    # The built-in macro, X_PATH, causes too many problems, these days everyone uses Xorg,
    # so we can ask pkg-config to find it for us.
    PKG_CHECK_MODULES(X11, x11, [], [drawstuff="none"])
fi

dnl Check for OpenGL
if test "x$drawstuff" = "xOSX"; then
  AC_DEFINE([HAVE_APPLE_OPENGL_FRAMEWORK], [1],
            [Use the Apple OpenGL framework.])
  GL_LIBS="-framework OpenGL -framework GLUT"
elif test "x$drawstuff" != "xnone"; then
  have_gl_headers=yes
  AC_CHECK_HEADERS(GL/gl.h GL/glu.h GL/glext.h, ,
        [have_gl_headers=no],
       [[#ifdef WIN32
         #include <windows.h>
         #endif
         #if HAVE_GL_GL_H
         #include <GL/gl.h>
         #endif
  	     #if HAVE_GL_GLU_H
         #include <GL/glu.h>
         #endif
       ]])
  have_gl=no
  have_glu=no
  TEMP_LDFLAGS="$LDFLAGS"
  AC_CHECK_LIB(GL, main, [GL_LIBS="-lGL"; have_gl=yes])
  AC_CHECK_LIB(GLU, main, [GL_LIBS="-lGLU $GL_LIBS"; have_glu=yes], , -lGL)
  AC_CHECK_LIB(opengl32, main, [GL_LIBS="-lopengl32"; have_gl=yes])
  AC_CHECK_LIB(glu32, main, [GL_LIBS="-lglu32 $GL_LIBS"; have_glu=yes], , -lopengl32)
  LDFLAGS="$TEMP_LDFLAGS"
  if test $have_gl = no -o $have_glu = no -o $have_gl_headers = no; then
    drawstuff="none"
  fi
fi
AC_SUBST(GL_LIBS)

dnl Set Conditionals
AM_CONDITIONAL(WIN32, test x$drawstuff = xWin32)
AM_CONDITIONAL(X11, test x$drawstuff = xX11)
AM_CONDITIONAL(OSX, test x$drawstuff = xOSX)
AM_CONDITIONAL(ENABLE_DRAWSTUFF, test x$drawstuff != xnone)

dnl Check if we want to build demos
AC_MSG_CHECKING(if demos should be built)
AC_ARG_ENABLE(demos,
        AS_HELP_STRING([--disable-demos], [don't build demos]),
        enable_demos=$enableval,enable_demos=yes)
if test x$drawstuff = xnone -a x$enable_demos = xyes ; then
    enable_demos=no
    AC_MSG_RESULT($enable_demos)
    AC_MSG_WARN([Demos will not be built because OpenGL doesn't seem to work. See `config.log' for details.])
else
    AC_MSG_RESULT($enable_demos)
fi


dnl stdc++ is required when linking C programs against ode
AC_CHECK_LIB(stdc++,main,[LIBSTDCXX="-lstdc++"],[LIBSTDCXX=])
AC_SUBST(LIBSTDCXX)
AC_CHECK_LIB(pthread,main,[LIBS="$LIBS -lpthread"])


dnl test if we will build demos
AM_CONDITIONAL(ENABLE_DEMOS, test x$enable_demos = xyes)


dnl Check if the user wants the old timesh collider
old_trimesh=no
AC_ARG_ENABLE([old-trimesh], AS_HELP_STRING([--enable-old-trimesh],[enable use of the old trimesh collider]),
        [old_trimesh=$enableval]
        )
if test x$old_trimesh = xyes -a $trimesh = opcode; then
        AC_DEFINE(dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER, 1,
                [Use the old trimesh-trimesh collider])
else
        old_trimesh=no
fi


dnl Check if the user wants to profile ODE using gprof
AC_MSG_CHECKING(for gprof)
AC_ARG_ENABLE([gprof],
        AS_HELP_STRING([--enable-gprof],[enable profiling with gprof]),
        gprof=$enableval,
        gprof=no)
if test "$gprof" != no
then
    CFLAGS="-pg $CFLAGS"
    CXXFLAGS="-pg $CXXFLAGS"
    AC_MSG_RESULT(enabled)
    AC_CHECK_LIB(gmon, main,[LIBS="$LIBS -lgmon"])
else
    AC_MSG_RESULT(no)
fi

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT32_T
AC_FUNC_OBSTACK
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T

dnl Check for autoscan sugested functions
AC_CHECK_LIB(m, [main])
AC_CHECK_LIB(sunmath, [main])
AC_CHECK_FUNCS([floor memmove memset sqrt sqrtf sinf cosf fabsf atan2f fmodf copysignf copysign snprintf vsnprintf gettimeofday isnan isnanf _isnan _isnanf __isnan __isnanf strchr strstr pthread_attr_setstacklazy])
AC_FUNC_ALLOCA 

AC_ARG_ENABLE([threading-intf],
        AS_HELP_STRING([--disable-threading-intf],
            [disable threading interface support (external implementations may not be assigned; overrides --enable-builtin-threading-impl)]
        ),
        threading_intf=$enableval,threading_intf=yes)
AC_ARG_ENABLE([ou],
        AS_HELP_STRING([--enable-ou],
            [use TLS for global caches (allows threaded collision checks for isolated spaces)]
        ),
        use_ou_tls=$enableval,use_ou_tls=no)
use_ou="no"
if test x$use_ou_tls = xyes -o x$threading_intf = xyes
then
    use_ou="yes"
fi
if test x$use_ou = xyes
then
    OU_NAMESPACE=odeou
    AC_CONFIG_COMMANDS_POST([export OU_NAMESPACE=odeou])
    AC_DEFINE([_OU_NAMESPACE],[odeou],[libou namespace for ODE])
    AC_DEFINE([dOU_ENABLED],[1],[Generic OU features are enabled])
    AC_DEFINE([dATOMICS_ENABLED],[1],[Atomic API of OU is enabled])
    if test x$use_ou_tls = xyes
    then
      AC_DEFINE([dTLS_ENABLED],[1],[Thread Local Storage API of OU is enabled])
    fi

    case "$host_os" in
      cygwin* | mingw*)
        targetos=_OU_TARGET_OS_WINDOWS
        ;;
      *qnx*)
        targetos=_OU_TARGET_OS_QNX
        ;;
      *apple* | *darwin*)
        targetos=_OU_TARGET_OS_MAC
        ;;
      *sunos*)
        targetos=_OU_TARGET_OS_SUNOS
        ;;
      *aix*)
        targetos=_OU_TARGET_OS_AIX
        ;;
      *)
        targetos=_OU_TARGET_OS_GENUNIX
        ;;
    esac

    if test $targetos = _OU_TARGET_OS_MAC
    then
        MAC_OS_X_VERSION=1000
        AC_CHECK_FUNC([OSAtomicAdd32Barrier], [MAC_OS_X_VERSION=1040])
        AC_CHECK_FUNC([OSAtomicAnd32OrigBarrier], [MAC_OS_X_VERSION=1050])
        AC_DEFINE_UNQUOTED(MAC_OS_X_VERSION, $MAC_OS_X_VERSION, [Mac OS X version setting for OU Library])
    fi
    
    if test $targetos = _OU_TARGET_OS_SUNOS
    then
        AC_CHECK_FUNC(atomic_inc_32_nv, [],
            [targetos=_OU_TARGET_OS_GENUNIX])
    fi
    
    AC_DEFINE_UNQUOTED(_OU_TARGET_OS, $targetos, [Target OS setting for OU Library])
fi
AC_CONFIG_SUBDIRS([ou])
AM_CONDITIONAL(ENABLE_OU, test x$use_ou = xyes)

if test x$threading_intf = xyes
then
    AC_ARG_ENABLE([builtin-threading-impl],
        AS_HELP_STRING([--enable-builtin-threading-impl],
            [include built-in multithreaded threading implementation (still must be created and assigned to be used)]
        ),
        use_builtin_threading_impl=$enableval,use_builtin_threading_impl=no)
    if test x$use_builtin_threading_impl = xyes
    then
        AC_DEFINE([dBUILTIN_THREADING_IMPL_ENABLED],[1],[Built-in multithreaded threading implementation included])
    fi
else
    AC_DEFINE([dTHREADING_INTF_DISABLED],[1],[Threading interface is disabled])
    use_builtin_threading_impl=no
fi

col_cylinder_cylinder=none
col_box_cylinder=default
col_capsule_cylinder=none
col_convex_box=none
col_convex_capsule=none
col_convex_cylinder=none
col_convex_sphere=default
col_convex_convex=default


libccd=no
libccd_all=no
AC_ARG_ENABLE(libccd, AS_HELP_STRING([--enable-libccd],
            [enable all libccd colliders (except box-cylinder)]),
            libccd_all=$enableval)
if test x$libccd_all = xyes
then
    col_cylinder_cylinder=libccd
    col_capsule_cylinder=libccd
    col_convex_box=libccd
    col_convex_capsule=libccd
    col_convex_cylinder=libccd
    col_convex_sphere=libccd
    col_convex_convex=libccd
    libccd=yes
fi


AC_ARG_WITH([cylinder-cylinder], AS_HELP_STRING([--with-cylinder-cylinder=@<:@none,libccd@:>@], [use specific collider for cylinder-cylinder]),
            col_cylinder_cylinder=$withval)

AC_ARG_WITH([box-cylinder],
        AS_HELP_STRING([--with-box-cylinder=@<:@default,libccd@:>@], [use specific collider for box-cylinder]),
            col_box_cylinder=$withval)

AC_ARG_WITH([capsule-cylinder], AS_HELP_STRING([--with-capsule-cylinder=@<:@none,libccd@:>@], [use specific collider for capsule-cylinder]),
            col_capsule_cylinder=$withval)

AC_ARG_WITH([convex-box], AS_HELP_STRING([--with-convex-box=@<:@none,libccd@:>@], [use specific collider for convex-box]),
            col_convex_box=$withval)

AC_ARG_WITH([convex-capsule], AS_HELP_STRING([--with-convex-capsule=@<:@none,libccd@:>@], [use specific collider for convex-capsule]),
            col_convex_capsule=$withval)

AC_ARG_WITH([convex-cylinder], AS_HELP_STRING([--with-convex-cylinder=@<:@none,libccd@:>@], [use specific collider for convex-cylinder]),
            col_convex_cylinder=$withval)

AC_ARG_WITH([convex-sphere], AS_HELP_STRING([--with-convex-sphere=@<:@default,libccd@:>@], [use specific collider for convex-sphere]),
            col_convex_sphere=$withval)

AC_ARG_WITH([convex-convex], AS_HELP_STRING([--with-convex-convex=@<:@default,libccd@:>@], [use specific collider for convex-convex]),
            col_convex_convex=$withval)

if test x$col_cylinder_cylinder = xlibccd -o \
    x$col_box_cylinder = xlibccd -o \
    x$col_capsule_cylinder = xlibccd -o \
    x$col_convex_box = xlibccd -o \
    x$col_convex_capsule = libccd -o \
    x$col_convex_cylinder = xlibccd -o \
    x$col_convex_sphere = libccd -o \
    x$col_convex_convex = libccd
then
    libccd=yes
fi

AC_CONFIG_SUBDIRS([libccd])

AM_CONDITIONAL(LIBCCD, test x$libccd != xno)
AM_CONDITIONAL(LIBCCD_BOX_CYL,          test x$col_box_cylinder = xlibccd)
AM_CONDITIONAL(LIBCCD_CYL_CYL,          test x$col_cylinder_cylinder = xlibccd)
AM_CONDITIONAL(LIBCCD_CAP_CYL,          test x$col_capsule_cylinder = xlibccd)
AM_CONDITIONAL(LIBCCD_CONVEX_BOX,       test x$col_convex_box = xlibccd)
AM_CONDITIONAL(LIBCCD_CONVEX_CAP,       test x$col_convex_capsule = xlibccd)
AM_CONDITIONAL(LIBCCD_CONVEX_CYL,       test x$col_convex_cylinder = xlibccd)
AM_CONDITIONAL(LIBCCD_CONVEX_SPHERE,    test x$col_convex_sphere = xlibccd)
AM_CONDITIONAL(LIBCCD_CONVEX_CONVEX,    test x$col_convex_convex = xlibccd)



AC_ARG_ENABLE([asserts],
        AS_HELP_STRING([--disable-asserts],
            [disables debug error checking]),
        asserts=$enableval,asserts=yes)
if test x$asserts = xno
then
    CPPFLAGS="$CPPFLAGS -DdNODEBUG"
    if test x$use_ou = xyes
    then
        CPPFLAGS="$CPPFLAGS -DNDEBUG"
    fi
fi


dnl include found system headers into config.h
AH_TOP([
#ifndef ODE_CONFIG_H
#define ODE_CONFIG_H
])
AH_BOTTOM([

#ifdef HAVE_ALLOCA_H
#include <alloca.h>
#endif
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif

/* an integer type that we can safely cast a pointer to and
 * from without loss of bits.
 */
typedef uintptr_t intP;


#ifdef dSINGLE
       #define dEpsilon  FLT_EPSILON
#else
       #define dEpsilon  DBL_EPSILON
#endif


#include "typedefs.h"


#endif /* #define ODE_CONFIG_H */
])

dnl Finally write our Makefiles
AC_CONFIG_FILES([
 Makefile
 drawstuff/Makefile
 drawstuff/src/Makefile
 drawstuff/dstest/Makefile
 include/Makefile
 include/drawstuff/Makefile
 include/ode/Makefile
 include/ode/version.h
 include/ode/precision.h
 ode/Makefile
 ode/src/Makefile
 ode/src/joints/Makefile
 ode/demo/Makefile
 OPCODE/Makefile
 OPCODE/Ice/Makefile
 GIMPACT/Makefile
 GIMPACT/include/Makefile
 GIMPACT/include/GIMPACT/Makefile
 GIMPACT/src/Makefile
 tests/Makefile
 tests/joints/Makefile
 tests/UnitTest++/Makefile
 tests/UnitTest++/src/Makefile
 tests/UnitTest++/src/Posix/Makefile
 tests/UnitTest++/src/Win32/Makefile
 ode-config
 ode.pc
 ])
AC_OUTPUT

chmod +x ode-config

BUILDDIR=`pwd`

dnl Print some useful information
echo "Configuration:"
echo "  Build  system type:      $build"
echo "  Host   system type:      $host"
echo "  Use double precision:    $usedouble"
echo "  Use drawstuff:           $drawstuff"
echo "  Demos enabled:           $enable_demos"
echo "  Use OPCODE:              $opcode"
echo "  Use GIMPACT:             $gimpact"
echo "  Custom colliders:"
echo "        cylinder-cylinder: $col_cylinder_cylinder"
echo "        box-cylinder:      $col_box_cylinder"
echo "        capsule-cylinder:  $col_capsule_cylinder"
echo "        convex-box:        $col_convex_box"
echo "        convex-capsule:    $col_convex_capsule"
echo "        convex-cylinder:   $col_convex_cylinder"
echo "        convex-sphere:     $col_convex_sphere"
echo "        convex-convex:     $col_convex_convex"
echo "  Is target a Pentium:     $pentium"
echo "  Is target x86-64:        $cpu64"
echo "  Use old opcode trimesh collider: $old_trimesh"
echo "  TLS for global caches:   $use_ou_tls"
echo "  Threading intf enabled:  $threading_intf"
echo "  Built-in threading included: $use_builtin_threading_impl"
echo "  Enable debug error check: $asserts"
echo "  Headers will be installed in $includedir/ode"
echo "  Libraries will be installed in $libdir"
echo "  Building in directory    $BUILDDIR"

