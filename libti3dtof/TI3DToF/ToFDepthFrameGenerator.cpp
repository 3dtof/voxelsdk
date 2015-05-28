/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <ToFDepthFrameGenerator.h>

#include <ToFCamera.h>

#define PARAM_AMPLITUDE_SCALING_FACTOR "amplitudeScalingFactor"
#define PARAM_DEPTH_SCALING_FACTOR "depthScalingFactor"

namespace Voxel
{
  
namespace TI
{

ToFDepthFrameGenerator::ToFDepthFrameGenerator(): 
  DepthFrameGenerator((TI_VENDOR_ID << 16) | DepthCamera::FRAME_DEPTH_FRAME, DepthCamera::FRAME_DEPTH_FRAME, 0, 1),
_amplitudeScalingFactor(-1), _depthScalingFactor(-1)
{
  _frameGeneratorParameters[PARAM_AMPLITUDE_SCALING_FACTOR] = SerializablePtr(new SerializableFloat());
  _frameGeneratorParameters[PARAM_DEPTH_SCALING_FACTOR] = SerializablePtr(new SerializableFloat());
}

bool ToFDepthFrameGenerator::setProcessedFrameGenerator(FrameGeneratorPtr &p)
{
  ToFFrameGeneratorPtr g = std::dynamic_pointer_cast<ToFFrameGenerator>(p);
  
  if(g)
  {
    _tofFrameGenerator = g;
    return true;
  }
  
  return false;
}

bool ToFDepthFrameGenerator::_onWriteConfiguration()
{
  if(_amplitudeScalingFactor < 0)
    return false;
  
  return true;
}

bool ToFDepthFrameGenerator::_onReadConfiguration()
{
  if(
    !get(PARAM_AMPLITUDE_SCALING_FACTOR, _amplitudeScalingFactor) ||
    !get(PARAM_DEPTH_SCALING_FACTOR, _depthScalingFactor))
    return false;
  
  return true;
}

bool ToFDepthFrameGenerator::setParameters(float amplitudeScalingFactor, float depthScalingFactor)
{
  if(_amplitudeScalingFactor == amplitudeScalingFactor && _depthScalingFactor == depthScalingFactor)
    return true;
  
  _amplitudeScalingFactor = amplitudeScalingFactor;
  _depthScalingFactor = depthScalingFactor;
  
  if(
    !_set(PARAM_AMPLITUDE_SCALING_FACTOR, _amplitudeScalingFactor) ||
    !_set(PARAM_DEPTH_SCALING_FACTOR, _depthScalingFactor))
    return false;
  
  return writeConfiguration();
}

template <typename T>
void scaleAndCopy(float *dest, const T *source, SizeType count, float scale)
{
  while(count--)
    (*dest++) = (*source++)*scale;
}


bool ToFDepthFrameGenerator::generate(const FramePtr &in, FramePtr &out)
{
  ToFRawFramePtr toFRawFramePtr = std::dynamic_pointer_cast<ToFRawFrame>(in);
  ToFRawIQFramePtr toFRawIQFramePtr = std::dynamic_pointer_cast<ToFRawIQFrame>(in);
  
  if(!toFRawFramePtr && !toFRawIQFramePtr)
  {
    logger(LOG_ERROR) << "ToFDepthFrameGenerator: Expecting ToFRawFrame or ToFRawIQFrame but got some other type for conversion to depth frame." << std::endl;
    return false;
  }
  
  if(toFRawIQFramePtr)
  {
    if(!_tofFrameGenerator)
    {
      logger(LOG_ERROR) << "ToFDepthFrameGenerator: Don't know how to handle ToFRawIQFrame." << std::endl;
      return false;
    }
    
    if(!_tofFrameGenerator->generate(toFRawIQFramePtr, _intermediate))
      return false;
    
    toFRawFramePtr = std::dynamic_pointer_cast<ToFRawFrame>(_intermediate);
    
    if(!toFRawFramePtr)
    {
      logger(LOG_ERROR) << "ToFDepthFrameGenerator: Expecting ToFRawFrame but got some other type for conversion to depth frame." << std::endl;
      return false;
    }
  }
  
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(out.get());
  
  if(!depthFrame)
  {
    depthFrame = new DepthFrame();
    out = FramePtr(depthFrame);
  }
  
  depthFrame->size = toFRawFramePtr->size;
  depthFrame->id = toFRawFramePtr->id;
  depthFrame->timestamp = toFRawFramePtr->timestamp;
  
  auto totalSize = depthFrame->size.width*depthFrame->size.height;
  
  depthFrame->depth.resize(depthFrame->size.width*depthFrame->size.height);
  depthFrame->amplitude.resize(depthFrame->size.width*depthFrame->size.height);
  
  // NOTE: Add more sizes as necessary
  if(toFRawFramePtr->phaseWordWidth() == 1)
    scaleAndCopy(depthFrame->depth.data(), toFRawFramePtr->phase(), depthFrame->depth.size(), _depthScalingFactor);
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
    
    scaleAndCopy(depthFrame->depth.data(), (uint16_t *)toFRawFramePtr->phase(), depthFrame->depth.size(), _depthScalingFactor);
  }
  else if(toFRawFramePtr->phaseWordWidth() == 4)
    scaleAndCopy(depthFrame->depth.data(), (uint32_t *)toFRawFramePtr->phase(), depthFrame->depth.size(), _depthScalingFactor);
  else
  {
    logger(LOG_ERROR) << "ToFCamera: Don't know how to convert ToF frame data with phase data element size in bytes = " << toFRawFramePtr->phaseWordWidth() << std::endl;
    return false;
  }
  
  // NOTE: Add more sizes as necessary
  if(toFRawFramePtr->amplitudeWordWidth() == 1)
    scaleAndCopy(depthFrame->amplitude.data(), toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), _amplitudeScalingFactor);
  else if(toFRawFramePtr->amplitudeWordWidth() == 2)
    scaleAndCopy(depthFrame->amplitude.data(), (uint16_t *)toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), _amplitudeScalingFactor);
  else if(toFRawFramePtr->amplitudeWordWidth() == 4)
    scaleAndCopy(depthFrame->amplitude.data(), (uint32_t *)toFRawFramePtr->amplitude(), depthFrame->amplitude.size(), _amplitudeScalingFactor);
  else
  {
    logger(LOG_ERROR) << "ToFCamera: Don't know how to convert ToF frame data with amplitude data element size in bytes = " << toFRawFramePtr->amplitudeWordWidth() << std::endl;
    return false;
  }
  
  return true;
}




} 
}