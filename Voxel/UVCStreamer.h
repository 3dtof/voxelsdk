/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCSTREAMER_H
#define VOXEL_UVCSTREAMER_H

#include "Streamer.h"
#include "UVC.h"

namespace Voxel
{

// Parts of this class are borrowed from http://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html
// TODO In GNU/Linux OS, this currently supports file read/write mode only and seeks YUYV as pixel format
class UVCStreamer: public Streamer
{
  Ptr<UVC> _uvc;
  
  bool _initialized = false;
  
  size_t _frameByteSize;
  
public:
  UVCStreamer(DevicePtr device);
  
  virtual bool isInitialized() { return _uvc && _initialized; }
  
  virtual bool initForCapture();
  virtual bool capture(RawDataFramePtr p);
  
  inline size_t getFrameByteSize() { return _frameByteSize; }
  
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes);
  
  virtual ~UVCStreamer() {}
};

}

#endif // VOXEL_UVCSTREAMER_H
