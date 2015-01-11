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

class RegionOfInterest: public FrameSize
{
public:
  size_t x, y;
};

class FrameRate
{
public:
  size_t numerator, denominator;
  
  inline float getFrameRate() const { return (denominator == 0)?0:(((float)numerator)/denominator); }
};
  
class VideoMode
{
public:
  FrameSize frameSize;
  FrameRate frameRate;
  
  inline float getFrameRate() const { return frameRate.getFrameRate(); }
};

class SupportedVideoMode: public VideoMode
{
public:
  uint8_t bytesPerPixel;
  
  SupportedVideoMode(size_t width, size_t height, size_t rateNumerator, size_t rateDenominator, uint8_t bytesPerPixel)
  {
    frameSize.width = width;
    frameSize.height = height;
    frameRate.numerator = rateNumerator;
    frameRate.denominator = rateDenominator;
    this->bytesPerPixel = bytesPerPixel;
  }
};
  
}

#endif // VOXEL_POINT_H
