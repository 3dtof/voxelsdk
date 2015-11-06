#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Please specify version as major.minor.patch"
  exit 0
fi

version=$1

mkdir -p ../packages/$version
cd build
rm -rf doc
touch ../Voxel/SWIG/Voxel.i
cmake -DCMAKE_BUILD_TYPE=Release ..
make clean
make -j2
. make_deb.sh
sudo dpkg -i *$version*.deb
cp *$version*.deb ../../packages/$version/
cd ../../packages/$version
tar -cvzf libvoxel-$version-all.tgz *.deb
