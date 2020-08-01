make clean
make

time ./bin/pr cases/case$1.txt sample.out
#gdb ./bin/pr cases/case$1.txt sample.out
