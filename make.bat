@echo off
cd src
c++ test.cpp main.cpp -o main.exe -std=c++11 -I "../include" -L "../libs" -lftd2xx -lftd2xx64
move main.exe ../build
cd ..
