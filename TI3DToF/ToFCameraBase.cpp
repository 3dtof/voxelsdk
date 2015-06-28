/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFCameraBase.h"

namespace Voxel
{
  
namespace TI
{
  
bool ToFCameraBase::_init()
{
  if(!DepthCamera::_init())
    return false;
  
  return true;
}


bool ToFCameraBase::_onReset()
{
  VideoMode m, m2;
  
  /* setFrameSize to set the right stream parameters for streamer */
  if(!getMaximumVideoMode(m) || !getFrameSize(m2.frameSize) || !getFrameRate(m2.frameRate) || !setFrameSize(m2.frameSize))
    return false;
  
  // Just ensure that the frame rate is not out of bounds
  if(m2.getFrameRate() > m.getFrameRate())
  {
    m2.frameRate = m.frameRate;
    
    if(!setFrameRate(m2.frameRate) || !setFrameSize(m2.frameSize))
      return false;
  }
  
  return true;
  
}


  
bool ToFCameraBase::_captureRawUnprocessedFrame(RawFramePtr &rawFrame)
{
  if(!isInitialized() || !_streamer->isRunning())
    return false;
  
  if(_streamer->capture(_rawDataFrame))
  {
    rawFrame = std::dynamic_pointer_cast<RawFrame>(_rawDataFrame);
    return true;
  }
  
  return false;
}

bool ToFCameraBase::_convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame)
{
  FramePtr p1 = std::dynamic_pointer_cast<Frame>(rawFrame);
  FramePtr p2 = std::dynamic_pointer_cast<Frame>(depthFrame);
  
  bool ret = _tofDepthFrameGenerator->generate(p1, p2);
  
  if(ret)
  {
    depthFrame = std::dynamic_pointer_cast<DepthFrame>(p2);
    return true;
  }
  else
    return false;
}


bool ToFCameraBase::_start()
{
  if(!isInitialized())
    return false;
  
  VideoMode m;
  
  if(!getFrameSize(m.frameSize))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get current frame size" << std::endl;
    return false;
  }

  if(!getFrameRate(m.frameRate))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get current frame rate" << std::endl;
  }
  
  logger(LOG_INFO) << "ToFCamera: Starting with " << m.frameSize.width << "x" << m.frameSize.height << "@" << m.getFrameRate() << "fps" << std::endl;
  
  if(!_initStartParams()) // Initialize parameters to starts streaming
    return false;
  
  // Start stream
  return _streamer->start();
}

bool ToFCameraBase::_initStartParams()
{
  float amplitudeNormalizingFactor, depthScalingFactor;
  
  if(!_getAmplitudeNormalizingFactor(amplitudeNormalizingFactor) || !_getDepthScalingFactor(depthScalingFactor))
    return false;
  
  if(!_tofDepthFrameGenerator->setParameters(amplitudeNormalizingFactor, depthScalingFactor))
  {
    logger(LOG_ERROR) << "ToFCameraBase: Could not set parameters to ToFDepthFrameGenerator" << std::endl;
    return false;
  }
  
  return true;
}


bool ToFCameraBase::_stop()
{
  if(!isInitialized())
    return false;
  
  return _streamer->stop();
}

  
}
}