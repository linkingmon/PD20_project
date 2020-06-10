CC=g++
LDFLAGS=-std=c++11 -O3 -lm
SOURCES=src/placement.cpp src/main.cpp src/placer.cpp src/router.cpp src/utils.cpp 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=pr
INCLUDES=src/cell.h src/net.h src/placement.h src/placer.h src/router.h src/utils.h src/struc.h src/congestion.h

all: $(SOURCES) bin/$(EXECUTABLE)

bin/$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o:  %.c  ${INCLUDES}
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o bin/$(EXECUTABLE)
