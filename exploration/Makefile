-include ../global.mk

OBJS= BitMap.o Exploration.o WFD_Opt.o GridConverter.o Grid.o GridGenerator.o

APPS= exploration_tester
LDFLAGS+= -lutils -lsensor_range -llog -lgridfastslam -lpthread -lconfigfile -lsensor_base -lscanmatcher -lpathplanner 

LIBS+= -lgridfastslam -lconfigfile
CPPFLAGS+= -I../sensor

-include ../build_tools/Makefile.generic-shared-object
