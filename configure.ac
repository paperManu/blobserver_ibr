#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.68)
AC_INIT([BLOBSERVER_PLUGIN_TEMPLATE],
    [0.1.0])
AC_CONFIG_MACRO_DIR([m4])
AC_PROG_LIBTOOL
AM_INIT_AUTOMAKE()
AC_CONFIG_HEADERS([config.h])

AC_SUBST([LIBBLOBSERVER_SO_VERSION], [0:6:0])
AC_SUBST([LIBBLOBSERVER_API_VERSION], [0.6])
AC_DEFINE(LIBBLOBSERVER_API_VERSION, ["0.6"], [API version])

# Check for C and C++ compilers
AC_PROG_CXX
AM_PROG_CC_C_O

AC_MSG_CHECKING([to see if compiler understands -std=gnu++0x])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([ ], [ ])], [
  CXXFLAGS="$CFLAGS -std=gnu++0x"
#  CXXFLAGS="-g3 -O0 -std=gnu++0x"
  AC_MSG_RESULT([yes])
], [
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([compiler does not understand -std=c++0x])
])

export PKG_CONFIG_PATH=$srcdir/pkgconfig:/usr/local/lib/pkgconfig:/usr/lib/pkgconfig

case "$host" in *-darwin* | *-macos10*)
  LIBTOOL="glibtool"
  CXXFLAGS+=" -D__Darwin"
  LDFLAGS+=" -undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CXXFLAGS+=" -I/usr/local/include -I/opt/local/include"
    #LDFLAGS+=" -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CXXFLAGS+=" -I/sw/include"
    #LDFLAGS+=" -L/sw/lib"
  fi
  # OSX does not include /usr/local/lib/pkgconfig path, so we need to
  # add it. 
  PKG_CONFIG_PATH=$srcdir/pkgconfig:/usr/local/lib/pkgconfig:/Library/Frameworks/GStreamer.framework/Libraries/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  LDFLAGS+=" -fPIC -L/usr/local/lib -L/usr/local/lib64"
  ;;
esac

# Check for header files
AC_HEADER_STDC

# Blobserver
PKG_CHECK_MODULES([BLOBSERVER], blobserver, have_blobserver=true, have_blobserver=false)
if test "x${have_blobserver}" = "xfalse" ; then
    AC_MSG_ERROR([Missing blobserver])
fi

# GLib
PKG_CHECK_MODULES([GLIB], glib-2.0, have_glib=true, have_glib=false)
if test "x${have_glib}" = "xfalse" ; then
    AC_MSG_ERROR([Missing glib])
fi

# OpenCV
PKG_CHECK_MODULES([OPENCV], [opencv >= 2.4])
if test "x${have_opencv}" = "xfalse" ; then
    AC_MSG_ERROR([Missing opencv])
fi

# LibAtom
PKG_CHECK_MODULES([LIBATOM], atom-0.2)
if test "x${have_atom}" = "xfalse" ; then
    AC_MSG_ERROR([Missing atom])
fi

# OpenImageIO
AC_LANG_PUSH([C++])
AC_CHECK_HEADER([OpenImageIO/imageio.h],
    [HAVE_IMAGEIO=true],
    [HAVE_IMAGEIO=false])
AM_CONDITIONAL(IMAGEIO, test "$HAVE_IMAGEIO" = "true")
AC_LANG_POP([C++])

# Cuda
ARCH=`uname -m`
if [[ $ARCH == "x86_64" ]];
then
    SUFFIX="64"
else
    SUFFIX=""
fi
AC_ARG_WITH([cuda],
            [--with-cuda=PATH   prefix where cuda is installed
            [default=/usr/local/cuda]])
if test -n "$with_cuda"
then
    CUDA_CFLAGS="-I$with_cuda/include"
    CUDA_LIBS="-lcudart"
    CUDA_LDFLAGS="-L$with_cuda/lib$SUFFIX"
    NVCC="$with_cuda/bin/nvcc"
else
    CUDA_CFLAGS="-I/usr/local/cuda/include"
    CUDA_LIBS="-lcudart"
    CUDA_LDFLAGS="-L/usr/local/cuda/lib$SUFFIX"
    NVCC="nvcc"
fi
AC_SUBST(CUDA_CFLAGS)
AC_SUBST(CUDA_LIBS)
AC_SUBST(CUDA_LDFLAGS)
AC_SUBST(NVCC)

AC_CHECK_PROG(HAVE_CUDA, nvcc, true, false)
AM_CONDITIONAL(CUDA, test "$HAVE_CUDA" = "true")
if test "$HAVE_CUDA" = "true"
then
    AC_DEFINE([HAVE_CUDA], [1], [Defined to 1 if CUDA is found])
else
    AC_DEFINE([HAVE_CUDA], [0], [Defined to 1 if CUDA is found])
fi

#-------------#
AC_CONFIG_FILES([
    Makefile
    src/Makefile
    ])
AC_OUTPUT

