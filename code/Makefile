UNAME_S := $(shell uname -s)
APP_NAME=wireroute

OBJS=wireroute.o validate.o

ifeq ($(UNAME_S), Darwin)
    CXX = g++-13
else
	CXX = g++
endif
CXXFLAGS = -Wall -std=c++17 -m64 -I. -fopenmp -Wno-unknown-pragmas

all: CXXFLAGS += -O3
all: $(APP_NAME)

debug: CXXFLAGS += -O0 -g
debug: $(APP_NAME)_debug

$(APP_NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

$(APP_NAME)_debug: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp %.h
	$(CXX) $(CXXFLAGS) -c $<

clean:
	/bin/rm -rf *~ *.o $(APP_NAME) $(APP_NAME)_debug *.class
