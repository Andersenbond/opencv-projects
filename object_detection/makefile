PROJECT_NAME = object_detection
CROSS_PREFIX= #arm-linux-gnueabihf-

AS = $(CROSS_PREFIX)as
CC = $(CROSS_PREFIX)g++
LD = $(CROSS_PREFIX)ld

CXXFLAGS += -DDEBUG -g
CCFLAGS += -DDEBUG -g

LDFLAGS=
LIBS=`pkg-config --cflags --libs opencv4`

OBJS = \
	$(PROJECT_NAME).o

$(PROJECT_NAME).o : $(PROJECT_NAME).cc
	$(CC) $(CFLAGS) -c $(PROJECT_NAME).cc -o $(PROJECT_NAME).o $(LIBS) $(CCFLAGS)

$(PROJECT_NAME) : $(OBJS)
	$(CC) $(OBJS) -o $(PROJECT_NAME)  $(LIBS) $(CCFLAGS)

all: $(PROJECT_NAME) 

.PHONY: clean
clean : 
	@echo $(OBJS) 
	-rm *.o
	-rm $(PROJECT_NAME)
