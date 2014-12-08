/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_STREAMER_H
#define VOXEL_STREAMER_H

#include "Device.h"
#include "Frame.h"
#include "VideoMode.h"

namespace Voxel
{

class Streamer
{
protected:
  DevicePtr _device;
  
  int _currentID = -1;
  TimeStampType _currentTimeStamp = 0;
  
public:
  Streamer(DevicePtr device): _device(device) {}
  
  virtual ~Streamer() {}
  
  // Call this before start of a capture sequence
  virtual bool initForCapture() = 0;
  virtual bool isInitialized() = 0;
  
  virtual bool capture(RawDataFramePtr p) = 0;
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes) = 0;
};

}

#endif // STREAMER_H
