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
/**
 * \addtogroup Frm
 * @{
 */
  
class FrameSize
{
public:
  uint32_t width, height;
  
  inline bool operator ==(const FrameSize &other) const
  {
    return width == other.width && height == other.height;
  }
  
  inline bool operator !=(const FrameSize &other) const
  {
    return !operator==(other);
  }
};

class RegionOfInterest: public FrameSize
{
public:
  uint32_t x, y;
  
  inline bool operator ==(const RegionOfInterest &other) const
  {
    return width == other.width && height == other.height && x == other.x && y == other.y;
  }
  
  inline bool operator !=(const RegionOfInterest &other) const
  {
    return !operator==(other);
  }
};

class FrameRate
{
public:
  uint32_t numerator, denominator;
  
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
  
  SupportedVideoMode(): bytesPerPixel(0)
  {
    frameSize.width = frameSize.height = 0;
    frameRate.numerator = 0; 
    frameRate.denominator = 1;
  }
  
  SupportedVideoMode(uint32_t width, uint32_t height, uint32_t rateNumerator, uint32_t rateDenominator, uint8_t bytesPerPixel)
  {
    frameSize.width = width;
    frameSize.height = height;
    frameRate.numerator = rateNumerator;
    frameRate.denominator = rateDenominator;
    this->bytesPerPixel = bytesPerPixel;
  }
};
/**
 * @}
 */

}

#endif // VOXEL_POINT_H
