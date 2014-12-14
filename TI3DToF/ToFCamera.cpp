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
  
  // Set parameters here
  VideoMode m;
  
  m.frameSize.width = 320;
  m.frameSize.height = 240;
  
  m.frameRate.numerator = 30;
  m.frameRate.denominator = 1;
    
  if(!_streamer->setVideoMode(m) || !setFrameRate(m.frameRate))
    return false;
  
  if(!_streamer->getCurrentVideoMode(m))// || !setFrameRate(m.frameRate))
    return false;
  
  log(INFO) << "Starting with " << m.frameSize.width << "x" << m.frameSize.height << "@" << m.getFrameRate() << "fps" << std::endl;
  
  if(!_initStartParams()) // Initialize parameters to starts streaming
    return false;
  
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