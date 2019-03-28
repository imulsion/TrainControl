@echo off
cd src
x86_64-w64-mingw32-g++ implementation.cpp main.cpp -o main.exe -std=c++11 -I "../include" -L "../libs" -lftd2xx -lftd2xx64
move main.exe ../build
cd ..
