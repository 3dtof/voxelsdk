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

/**
 * \addtogroup IO
 * @{
 */

  
// Parts of this class are borrowed from http://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html
// TODO This currently seeks only YUYV as pixel format
class VOXEL_EXPORT UVCStreamer : public Streamer
{
protected:
  class UVCStreamerPrivate;
  Ptr<UVCStreamerPrivate> _uvcStreamerPrivate;

  Ptr<VideoMode> _currentVideoMode;
  void _storeCurrentVideoMode(const VideoMode &videoMode);
  
  bool _uvcInit();
  bool _initForCapture();
  
  virtual bool _start();
  virtual bool _capture(RawDataFramePtr &p);
  virtual bool _stop();
  
public:
  UVCStreamer(DevicePtr device);
  
  virtual bool isInitialized();
  
  virtual bool getSupportedVideoModes(Vector<VideoMode> &videoModes);
  
  virtual bool getCurrentVideoMode(VideoMode &videoMode);
  virtual bool setVideoMode(const VideoMode &videoMode);
  
  virtual ~UVCStreamer();
};
/**
 * @}
 */

}

#endif // VOXEL_UVCSTREAMER_H
