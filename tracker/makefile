PROJECT_NAME = tracker
CROSS_PREFIX= #arm-linux-gnueabihf-

AS = $(CROSS_PREFIX)as
CC = $(CROSS_PREFIX)g++
LD = $(CROSS_PREFIX)ld

CXXFLAGS += -DDEBUG -g
CCFLAGS += -DDEBUG -g

LDFLAGS=
LIBS=`pkg-config --cflags --libs opencv4`

OBJS = \
	tracker.o

tracker.o : tracker.cc
	$(CC) $(CFLAGS) -c tracker.cc -o tracker.o $(LIBS) $(CCFLAGS)

$(PROJECT_NAME) : $(OBJS)
	$(CC) $(OBJS) -o tracker $(LIBS) $(CCFLAGS)

all: tracker

.PHONY: clean
clean : 
	@echo $(OBJS) 
	-rm *.o
	-rm $(PROJECT_NAME)
