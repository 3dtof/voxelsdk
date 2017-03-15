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
#include "Timer.h"

namespace Voxel
{
/**
 * \addtogroup IO
 * @{
 */


class VOXEL_EXPORT Streamer
{
protected:
  DevicePtr _device;
  
  Timer _time;
  
  int _currentID;
  TimeStampType _currentTimeStamp;
  
  bool _isRunning;
  
  virtual bool _start() = 0;
  virtual bool _capture(RawDataFramePtr &p) = 0;
  virtual bool _stop() = 0;
  
public:
  Streamer(DevicePtr device): _device(device) {
    _currentID = -1;
    _currentTimeStamp = 0;
    
    _isRunning = false;
  }
  
  virtual ~Streamer();
  
  virtual bool isInitialized() = 0;
  virtual bool isRunning() { return _isRunning; }
  
  virtual bool start();
  virtual bool capture(RawDataFramePtr &p);
  virtual bool stop();
  
  virtual bool getSupportedVideoModes(tVector<VideoMode> &videoModes) = 0;
  
  virtual bool getCurrentVideoMode(VideoMode &videoMode) = 0;
  virtual bool setVideoMode(const VideoMode &videoMode) = 0;
};
/**
 * @}
 */

}

#endif // STREAMER_H
