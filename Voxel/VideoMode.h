/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_VIDEO_MODE_H
#define VOXEL_VIDEO_MODE_H

namespace Voxel
{
  
class VideoMode
{
public:
  enum FrameRate
  {
    FRAME_RATE_30FPS,
    FRAME_RATE_25FPS,
    FRAME_RATE_24FPS,
    FRAME_RATE_23_997FPS
  };
  
  size_t frameSize[2];
  FrameRate frameRate;
};
  
}

#endif // VOXEL_POINT_H
