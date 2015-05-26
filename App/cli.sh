#!/bin/bash

P="`dirname $0`"
TI3DTOF_PATH="$P/../../libti3dtof/build/TI3DToF"

export VOXEL_FW_PATH="$TI3DTOF_PATH"
export VOXEL_CONF_PATH="$TI3DTOF_PATH"
export VOXEL_LIB_PATH="$TI3DTOF_PATH"

App/VoxelCLI
