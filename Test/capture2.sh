#!/bin/bash

export VOXEL_FW_PATH="$PWD/TI3DToF"
export VOXEL_CONF_PATH="$PWD/TI3DToF"
export VOXEL_LIB_PATH="$PWD/TI3DToF"

Test/CameraSystemTest -v 0x0451 -p 0x9102,0x9103 -f temp.out
