### You should not need to change anything below.
LINUX=1
MACOSX=0

# Compilers
CC=gcc
CXX=g++

# Paths
MAPPING_ROOT=/home/matan/workspace/Gmapping_Original/trunk
LIBDIR=/home/matan/workspace/Gmapping_Original/trunk/lib
BINDIR=/home/matan/workspace/Gmapping_Original/trunk/bin

# Build tools
PRETTY=/home/matan/workspace/Gmapping_Original/trunk/build_tools/pretty_compiler
MESSAGE=/home/matan/workspace/Gmapping_Original/trunk/build_tools/message
TESTLIB=/home/matan/workspace/Gmapping_Original/trunk/build_tools/testlib

# QT support
MOC=moc-qt3
QT_LIB=-lqt-mt
QT_INCLUDE=-I/usr/include/qt3

# ARIA support
ARIA_LIB=
ARIA_INCLUDE=


# # KDE support
# KDE_LIB=
# KDE_INCLUDE=
# UIC=

# Generic makefiles
MAKEFILE_GENERIC=/home/matan/workspace/Gmapping_Original/trunk/build_tools/Makefile.generic-shared-object
MAKEFILE_APP=/home/matan/workspace/Gmapping_Original/trunk/build_tools/Makefile.app
MAKEFILE_SUBDIRS=/home/matan/workspace/Gmapping_Original/trunk/build_tools/Makefile.subdirs


# Flags
CPPFLAGS+=-DLINUX -I/home/matan/workspace/Gmapping_Original/trunk 
CXXFLAGS+=
LDFLAGS+= -Xlinker -rpath /home/matan/workspace/Gmapping_Original/trunk/lib
CARMENSUPPORT=0
ARIASUPPORT=0



include /home/matan/workspace/Gmapping_Original/trunk/manual.mk

