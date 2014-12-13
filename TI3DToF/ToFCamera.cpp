/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFCamera.h"

namespace Voxel
{
  
namespace TI
{
  
bool ToFCamera::_captureDepthFrame(RawFramePtr &rawFrame, DepthFramePtr &depthFrame)
{
  return false;
}

bool ToFCamera::_captureRawFrame(RawFramePtr &rawFrame)
{
  if(!isInitialized() || !_streamer->isRunning())
    return false;
  
  RawDataFramePtr r;
  
  bool ret = _streamer->capture(r);
  
  if(ret)
  {
    rawFrame = std::dynamic_pointer_cast<RawFrame, RawDataFrame>(r);
    return true;
  }
  else
    return false;
}

bool ToFCamera::_start()
{
  if(!isInitialized())
    return false;
  
  if(!_initStartParams()) // Initialize parameters to starts streaming
    return false;
  
  // Set parameters here
  VideoMode m;
  if(_streamer->getCurrentVideoMode(m))
  {
    m.frameSize.width = 320;
    m.frameSize.height = 240;
    _streamer->setVideoMode(m);
  }
  
  // Start stream
  return _streamer->start();
}

bool ToFCamera::_stop()
{
  if(!isInitialized())
    return false;
  
  return _streamer->stop();
}

  
}
}