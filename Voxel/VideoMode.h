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
  
class FrameSize
{
public:
  size_t width, height;
};

class FrameRate
{
public:
  size_t numerator, denominator;
};
  
class VideoMode
{
public:
  FrameSize frameSize;
  FrameRate frameRate;
  
  inline float getFrameRate() { return (frameRate.denominator == 0)?0:(((float)frameRate.numerator)/frameRate.denominator); }
};
  
}

#endif // VOXEL_POINT_H
