#!/bin/bash

export VOXEL_FW_PATH="$PWD/../TI3DToF"
Test/DownloaderTest -v 0x0451 -p 0x9102 -f ../TI3DToF/OPT9220_0v27.fw
sleep 2

# Illumination settings
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x2D0E -d     0x14 -o
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x2D0F -d     0xFF -o
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x2D0D -d     0x54 -o
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x2D0E -d     0x94 -o

# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5831 -d      0x3 -t 24 -b  3 -l  1 -o #ramppat
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3C -d      0x1 -t 24 -b  2 -l  2 -o #block header enable
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3C -d    0x400 -t 24 -b 23 -l  4 -o #block size
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C40 -d      0x1 -t 24 -b 14 -l 14 -o #op cs
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3F -d      0x1 -t 24 -b  0 -l  0 -o #fb ready
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5882 -d  0x1d4c0 -t 24 -b 21 -l  0 -o #pix count max
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5880 -d      0x1 -t 24 -b  0 -l  0 -o #tg enable
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C35 -d      0x1 -t 24 -b 23 -l 23 -o #illum_en_pol
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x584c -d      0x1 -t 24 -b 23 -l 23 -o #easy conf
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x584c -d     0x14 -t 24 -b  6 -l  0 -o #integration duty cycle
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C31 -d      0x2 -t 24 -b  8 -l  4 -o #amplitude scale
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C31 -d     0x19 -t 24 -b 13 -l  9 -o #frquency scale
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5835 -d     0x10 -t 24 -b  4 -l  0 -o #mod_ps1
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5834 -d      0x1 -t 24 -b  0 -l  0 -o #mod_pll 1
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5834 -d      0x0 -t 24 -b  0 -l  0 -o #mod_pll 0
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C83 -d      0x4 -t 24 -b  7 -l  4 -o #sub frames
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x584c -d     0x14 -t 24 -b  6 -l  0 -o #integration duty cycle

# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3C -d      0x1 -t 24 -b  2 -l  2 -i #block header enable
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3C -d    0x400 -t 24 -b 23 -l  4 -i #block size
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C40 -d      0x1 -t 24 -b 14 -l 14 -i #op cs
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C3F -d      0x1 -t 24 -b  0 -l  0 -i #fb ready
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5882 -d  0x1d4c0 -t 24 -b 21 -l  0 -i #pix count max
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5880 -d      0x1 -t 24 -b  0 -l  0 -i #tg enable
# Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C35 -d      0x1 -t 24 -b 23 -l 23 -i #illum_en_pol

# Debug enable
Test/Voxel14RegisterTest -v 0x0451 -p 0x9103 -r 0x5C29 -d      0x1 -t 24 -b  1 -l  1 -o # debug en
Test/UVCStreamerTest -v 0x0451 -p 0x9103 -f temp.out