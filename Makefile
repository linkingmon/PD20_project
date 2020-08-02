#all: flute/flute-net 

#flute/flute-net: flute/flute-net.c ${OBJ}
#	$(CC) $(CFLAGS) -o flute/flute-net.c ${OBJ} $@ -lm

#flute/flute.o: flute/flute.c flute/flute.h
#	$(CC) $(CFLAGS) -c -o flute.o flute.c

#flute/flute_mst.o: flute/flute_mst.c flute/flute.h
#	$(CC) $(CFLAGS) -c -o flute_mst.o flute_mst.c

CC=g++
LDFLAGS=-std=c++11 -O3 -lm -g
SOURCES=src/placement.cpp src/main.cpp src/placer.cpp src/router.cpp src/utils.cpp src/mst.cpp src/maze_router.cpp flute/flute.c flute/dist.c flute/dl.c flute/err.c flute/heap.c flute/mst2.c flute/neighbors.c \
	flute/bookshelf_IO.c flute/memAlloc.c flute/flute_mst.c 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=pr
INCLUDES=src/cell.h src/net.h src/placement.h src/placer.h src/router.h src/utils.h src/struc.h src/congestion.h src/mst.h src/myUsage.h src/maze_router.h\ src/MinHeap.h flute/flute.h LayerAssign.h

all: $(SOURCES) bin/$(EXECUTABLE)

bin/$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o:  %.c ${INCLUDES}
	$(CC) $(CFLAGS) $< -o $@ -L../flute
#$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf *.o bin/$(EXECUTABLE)


