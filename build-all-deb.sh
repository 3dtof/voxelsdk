#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify version as major.minor.patch"
  exit 0
fi

version=$1

mkdir -p ../packages/$version
mkdir -p build; cd build
rm -rf doc

if [ -e ../Voxel/SWIG/Voxel.i ]; then touch ../Voxel/SWIG/Voxel.i; fi

cmake -DCMAKE_BUILD_TYPE=Release -DGENERATE_PYTHON_BINDINGS=FALSE ..
make clean
make -j2
. make_deb.sh
sudo dpkg -i *$version*.deb
cp *$version*.deb ../../packages/$version/
cd ../../packages/$version
tar -cvzf libvoxel-$version-all.tgz *.deb
