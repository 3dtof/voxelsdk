#!/bin/sh

version=$1

mkdir -p ../Releases/

touch Voxel/SWIG/Voxel.i

cd build
#cmake --build . --target clean --config Release
cmake --build . --target PACKAGE --config Release

cp Voxel*SDK*$version*.exe ../../Releases
