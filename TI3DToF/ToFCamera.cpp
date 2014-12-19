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
  
bool ToFCamera::_captureRawUnprocessedFrame(RawFramePtr &rawFrame)
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


template <typename T>
void scaleAndCopy(float *dest, const T *source, SizeType count, float scale)
{
  while(count--)
    (*dest++) = (*source++)*scale;
}

bool ToFCamera::_convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame)
{
  ToFRawFramePtr toFRawFramePtr = std::dynamic_pointer_cast<ToFRawFrame>(rawFrame);
  
  if(!toFRawFramePtr)
  {
    logger(ERROR) << "ToFCamera: Expecting ToFRawFrame but got some other type for conversion to depth frame." << std::endl;
    return false;
  }
  
  if(!depthFrame)
    depthFrame = DepthFramePtr(new DepthFrame());
  
  depthFrame->size = toFRawFramePtr->size;
  depthFrame->id = toFRawFramePtr->id;
  depthFrame->timestamp = toFRawFramePtr->timestamp;
  
  auto totalSize = depthFrame->size.width*depthFrame->size.height;
  
  depthFrame->depth.resize(depthFrame->size.width*depthFrame->size.height);
  depthFrame->amplitude.resize(depthFrame->size.width*depthFrame->size.height);
  
  float amplitudeNormalizingFactor, depthScalingFactor;
  
  if(!_getAmplitudeNormalizingFactor(amplitudeNormalizingFactor) or !_getDepthScalingFactor(depthScalingFactor))
    return false;
  
  // NOTE: Add more sizes as necessary
  if(toFRawFramePtr->phaseWordWidth() == 1)
    scaleAndCopy(depthFrame->depth.data(), toFRawFramePtr->phase(), depthFrame->depth.size(), depthScalingFactor);
  else if(toFRawFramePtr->phaseWordWidth() == 2)
  {
    // Histogram printing code in comment block. Uncomment to see the histogram per frame (8 bins)
//     uint hist[8];
//     
//     for(auto i = 0; i < 8; i++)
//       hist[i] = 0;
//     
//     uint16_t *p = (uint16_t *)toFRawFramePtr->phase();
//     
//     for(auto i = 0; i < depthFrame->depth.size(); i++)
//       hist[p[i] % 8]++;
//     
//     logger(INFO) << "Hist: " << depthFrame->depth.size() << " -> [ ";
//     for(auto i = 0; i < 8; i++)
//       logger << hist[i] << " ";
//     logger << "]" << std::endl;
    
    scaleAndCopy(depthFrame->depth.data(), (uint16_t *)toFRawFramePtr->phase(), depthFrame->depth.size(), depthScalingFactor);
  }
  else if(toFRawFramePtr->phaseWordWidth() == 4)
    scaleAndCopy(depthFrame->depth.data(), (uint32_t *)toFRawFramePtr->phase(), depthFrame->depth.size(), depthScalingFactor);
  else
  {
    logger(ERROR) << "ToFCamera: Don't know how to convert ToF frame data with phase data element size in bytes = " << toFRawFramePtr->phaseWordWidth() << std::endl;
    return false;
  }
  
  // NOTE: Add more sizes as necessary
  if(toFRawFramePtr->amplitudeWordWidth() == 1)
    scaleAndCopy(depthFrame->amplitude.data(), toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), amplitudeNormalizingFactor);
  else if(toFRawFramePtr->amplitudeWordWidth() == 2)
    scaleAndCopy(depthFrame->amplitude.data(), (uint16_t *)toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), amplitudeNormalizingFactor);
  else if(toFRawFramePtr->amplitudeWordWidth() == 4)
    scaleAndCopy(depthFrame->amplitude.data(), (uint32_t *)toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), amplitudeNormalizingFactor);
  else
  {
    logger(ERROR) << "ToFCamera: Don't know how to convert ToF frame data with amplitude data element size in bytes = " << toFRawFramePtr->amplitudeWordWidth() << std::endl;
    return false;
  }
  
  return true;
}


bool ToFCamera::_start()
{
  if(!isInitialized())
    return false;
  
  // Set parameters here
  VideoMode m;
  
  m.frameSize.width = 640;
  m.frameSize.height = 240;
  
  m.frameRate.numerator = 25;
  m.frameRate.denominator = 1;
    
  if(!_streamer->setVideoMode(m) || !setFrameRate(m.frameRate))
    return false;
  
  if(!_streamer->getCurrentVideoMode(m))// || !setFrameRate(m.frameRate))
    return false;
  
  logger(INFO) << "Starting with " << m.frameSize.width << "x" << m.frameSize.height << "@" << m.getFrameRate() << "fps" << std::endl;
  
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