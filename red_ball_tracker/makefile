PROJECT_NAME = red_ball_tracker
CROSS_PREFIX= #arm-linux-gnueabihf-

AS = $(CROSS_PREFIX)as
CC = $(CROSS_PREFIX)g++
LD = $(CROSS_PREFIX)ld

CXXFLAGS += -DDEBUG -g
CCFLAGS += -DDEBUG -g

LDFLAGS=
LIBS=`pkg-config --cflags --libs opencv4`

OBJS = \
	red_ball_tracker.o

red_ball_tracker.o : red_ball_tracker.cc
	$(CC) $(CFLAGS) -c red_ball_tracker.cc -o red_ball_tracker.o $(LIBS) $(CCFLAGS)

$(PROJECT_NAME) : $(OBJS)
	$(CC) $(OBJS) -o red_ball_tracker $(LIBS) $(CCFLAGS)

all: red_ball_tracker

.PHONY: clean
clean : 
	@echo $(OBJS) 
	-rm *.o
	-rm $(PROJECT_NAME)
