#!/bin/bash

mkdir -p build; cd build
rm -rf doc

if [ -e ../Voxel/SWIG/Voxel.i ]; then touch ../Voxel/SWIG/Voxel.i; fi

cmake -DCMAKE_BUILD_TYPE=Release -DGENERATE_PYTHON_BINDINGS=FALSE ..
make clean
make -j`nproc`
. make_deb.sh
sudo dpkg -i *.deb
