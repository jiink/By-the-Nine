#!/bin/bash

#Delete unneeded files
rm *.o
rm *.out

echo "Compiling c file"
gcc -m64 -fno-pie -no-pie -c -std=c17 -o chip9.o chip9.c -g

echo "Linking the object files"
g++ -m64 -fno-pie -no-pie -o chip9.out -std=c++17 chip9.o -g

echo "Running executable"
echo ""
gdb ./chip9.out

echo "Bash script will terminate"
