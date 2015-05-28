#!/bin/bash

export CONF="Debug"

P="`dirname $0`"
TI3DTOF_PATH="$P/../../libti3dtof/build/TI3DToF"

export VOXEL_FW_PATH="$TI3DTOF_PATH/$CONF"
export VOXEL_CONF_PATH="$TI3DTOF_PATH/$CONF"
export VOXEL_LIB_PATH="$TI3DTOF_PATH/$CONF"


export PATH="$P/../../libvoxel/build/Voxel/$CONF":$PATH
export PATH="$P/../../libvoxelpcl/build/VoxelPCL/$CONF":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/Boost/lib64-msvc-12.0":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/FLANN/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/Qhull/bin":$PATH
export PATH="/C/Users/x0227529/Documents/PCL 1.7.2/3rdParty/VTK/bin":$PATH

App/$CONF/VoxelCLI