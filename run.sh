make clean
make

time ./bin/pr cases/case$1.txt output/case$1.out
#gdb ./bin/pr cases/case$1.txt output/case$1.out
