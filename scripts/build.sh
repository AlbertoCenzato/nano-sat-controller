#!/bin/bash

install_dir="/home/pi/NanoSatController"

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -D CMAKE_INSTALL_PREFIX=$install_dir .. && \
make -j3

mkdir -p $install_dir
cp NanoSatController.exe $install_dir/NanoSatController.exe
cp -r ../resources $install_dir
echo "NanoSatController installed in $install_dir"

# TODO: activate camera driver 
# sudo modprobe bcm-qualcosa