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
    CAPTURE_USER_POINTER,
    CAPTURE_STREAMING // This is an intermediate which will be used to decide whether CAPTURE_MMAP or CAPTURE_USER_POINTER can be used
  } _captureMode;

  Ptr<UVC> _uvc;
  
  bool _initialized = false;
  
  size_t _frameByteSize;
  
  Vector<UVCRawData> _rawDataBuffers;
  
  bool _uvcInit();
  bool _initForCapture();
  inline void _updateFrameByteSize(uint32_t width, uint32_t height, uint32_t bytesPerLine, uint32_t frameSize);
  
  virtual bool _start();
  virtual bool _capture(RawDataFramePtr &p);
  virtual bool _stop();
  
public:
  UVCStreamer(DevicePtr device);
  
  virtual bool isInitialized() { return _uvc && _initialized; }
  
  inline size_t getFrameByteSize() { return _frameByteSize; }
  
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes);
  
  virtual bool getCurrentVideoMode(VideoMode &videoMode);
  virtual bool setVideoMode(const VideoMode &videoMode);
  
  virtual ~UVCStreamer();
};

void UVCStreamer::_updateFrameByteSize ( uint32_t width, uint32_t height, uint32_t bytesPerLine, uint32_t frameSize )
{
  /* Buggy driver paranoia. */
  size_t min = width * 2;
  if(bytesPerLine < min)
    bytesPerLine = min;
  
  min = bytesPerLine * height;
  
  if(frameSize < min)
    _frameByteSize = min;
  else
    _frameByteSize = frameSize;
}


}

#endif // VOXEL_UVCSTREAMER_H
