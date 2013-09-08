AM_CPPFLAGS = \
    -DG_LOG_DOMAIN=""

pluginsdir = $(prefix)/lib/blobserver-@LIBBLOBSERVER_API_VERSION@

plugins_LTLIBRARIES = libactuatortemplate.la

libactuatortemplate_la_SOURCES = \
	template.cpp

libactuatortemplate_la_CXXFLAGS = \
	$(BLOBSERVER_CFLAGS) \
	$(GLIB_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(LIBATOM_CFLAGS)
