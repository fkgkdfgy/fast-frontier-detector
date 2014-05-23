-include ./global.mk

ifeq ($(CARMENSUPPORT),1)
SUBDIRS=utils  sensor log configfile scanmatcher carmenwrapper gridfastslam pathplanner exploration wolfram gui gfs-carmen 
else
ifeq ($(MACOSX),1)
SUBDIRS=utils sensor log configfile  scanmatcher gridfastslam  pathplanner exploration wolfram
else
SUBDIRS=utils sensor log configfile  scanmatcher gridfastslam  pathplanner exploration wolfram gui 
endif
endif

LDFLAGS+=
CPPFLAGS+= -I../sensor

-include ./build_tools/Makefile.subdirs

