/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_VIDEO_MODE_H
#define VOXEL_VIDEO_MODE_H

#include <stdint.h>

namespace Voxel
{
  
class VideoMode
{
public:
  size_t frameSize[2];
  size_t frameRate[2]; // [0] -> numerator, [1] -> denominator
  
  inline float getFrameRate() { return (frameRate[1] == 0)?0:(frameRate[0]/frameRate[1]); }
};
  
}

#endif // VOXEL_POINT_H
