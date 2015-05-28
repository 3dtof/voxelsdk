#!/bin/bash

export CONF="Release"

export PATH="$PWD/Voxel/$CONF":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/Boost/lib64-msvc-12.0":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/FLANN/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/Qhull/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/VTK/bin":$PATH
export VOXEL_FW_PATH="$PWD/TI3DToF/$CONF"
export VOXEL_CONF_PATH="$PWD/TI3DToF/$CONF"
export VOXEL_LIB_PATH="$PWD/TI3DToF/$CONF"

App/$CONF/SimpleVoxelViewer