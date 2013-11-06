include $(top_srcdir)/cuda.mk

AM_CPPFLAGS = \
    -DG_LOG_DOMAIN=""

pluginsdir = $(prefix)/lib/blobserver-@LIBBLOBSERVER_API_VERSION@
libdir = $(prefix)/lib/blobserver-@LIBBLOBSERVER_API_VERSION@/thrust

if CUDA
LINK = $(LIBTOOL) --mode=link $(CC) -o $@ $(CUDA_LDFLAGS) $(CUDA_LIBS)
EXTRA_DIST = accumulator.cu
lib_LTLIBRARIES = libaccumulator.la
libaccumulator_la_SOURCES = accumulator.cu
endif

if IMAGEIO
plugins_LTLIBRARIES = libactuatoribr.la

libactuatoribr_la_SOURCES = \
	ibr.cpp

libactuatoribr_la_CXXFLAGS = \
	$(BLOBSERVER_CFLAGS) \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)

libactuatoribr_la_LDFLAGS = \
	$(top_builddir)/src/libaccumulator.la \
	-lm $(CUDA_LDFLAGS) $(CUDA_LIBS) \
	-lOpenImageIO
endif
