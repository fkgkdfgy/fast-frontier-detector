-include ../global.mk

OBJS= node.o pathplan2.o 

APPS= example
LDFLAGS+= -lutils -lsensor_range -llog -lgridfastslam -lpthread -lconfigfile -lsensor_base -lscanmatcher 

LIBS+= -lgridfastslam -lconfigfile 
CPPFLAGS+= -I../sensor

-include ../build_tools/Makefile.generic-shared-object
