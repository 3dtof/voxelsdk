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
// TODO This currently seeks only YUYV as pixel format
class UVCStreamer: public Streamer
{
protected:
  enum _CaptureMode
  {
    CAPTURE_READ_WRITE,
    CAPTURE_MMAP,
    CAPTURE_USER_POINTER
  } _captureMode;

  Ptr<UVC> _uvc;
  
  bool _initialized = false;
  
  size_t _frameByteSize;
  
  VideoMode _currentVideoMode;
  
  Vector<UVCRawData> _rawDataBuffers;
  
  bool _uvcInit();
  bool _initForCapture();
  
  virtual bool _start();
  virtual bool _capture(RawDataFramePtr &p);
  virtual bool _stop();
  
public:
  UVCStreamer(DevicePtr device);
  
  virtual bool isInitialized() { return _uvc && _initialized; }
  
  inline size_t getFrameByteSize() { return _frameByteSize; }
  
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes);
  
  virtual const VideoMode &getCurrentVideoMode();
  
  virtual ~UVCStreamer();
};

}

#endif // VOXEL_UVCSTREAMER_H
