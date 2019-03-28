@echo off
cd src
g++ implementation.cpp main.cpp -o main.exe -std=c++11 -I "../include" -L "../libs" -lftd2xx -lftd2xx64
move main.exe ../build
cd ..
