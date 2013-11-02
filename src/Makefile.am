AM_CPPFLAGS = \
    -DG_LOG_DOMAIN=""

pluginsdir = $(prefix)/lib/blobserver-@LIBBLOBSERVER_API_VERSION@

plugins_LTLIBRARIES = libactuatoribr.la

libactuatoribr_la_SOURCES = \
	ibr.cpp

libactuatoribr_la_CXXFLAGS = \
	$(BLOBSERVER_CFLAGS) \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)
