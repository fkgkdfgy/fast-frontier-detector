-include ../global.mk

OBJS= FrontierExtractor.o

#APPS= exploration_tester
#LDFLAGS+= -lscanmatcher
#LDFLAGS+= -lutils -lsensor_range -llog -lgridfastslam -lpthread -lconfigfile -lsensor_base -lscanmatcher -lpathplanner
LDFLAGS+= -lutils -lsensor_range -llog -lgridfastslam -lpthread -lconfigfile -lsensor_base -lscanmatcher 

LIBS+= -lgridfastslam -lconfigfile
CPPFLAGS+= -I../sensor

-include ../build_tools/Makefile.generic-shared-object
