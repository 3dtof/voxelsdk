/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <ToFCameraFactoryBase.h>
#include <Logger.h>
#include <ToFCamera.h>

namespace Voxel
{
  
namespace TI
{
  

ToFCameraFactoryBase::ToFCameraFactoryBase(const String &name): DepthCameraFactory(name)
{
}

bool ToFCameraFactoryBase::getChannels(Device &device, Vector<int> &channels)
{
  channels.resize(1);
  channels[0] = 0; // This supports only one channel for all supported devices
  return true;
}

Vector<GeneratorIDType> ToFCameraFactoryBase::getSupportedGeneratorTypes()
{
  Vector<GeneratorIDType> d(2);
  d[0] = ((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_DEPTH_FRAME;
  d[1] = ((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_RAW_FRAME_PROCESSED;
  return d;
}

bool ToFCameraFactoryBase::getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator)
{
  if(generatorID == (((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_DEPTH_FRAME))
  {
    if(frameType != DepthCamera::FRAME_DEPTH_FRAME)
      return false;
      
    frameGenerator = FrameGeneratorPtr(new ToFDepthFrameGenerator());
    return true;
  }
  else if(generatorID == (((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_RAW_FRAME_PROCESSED))
  {
    if(frameType != DepthCamera::FRAME_RAW_FRAME_PROCESSED)
      return false;
    
    frameGenerator = FrameGeneratorPtr(new ToFFrameGenerator());
    return true;
  }
  else
  {
    logger(LOG_ERROR) << "ToFCameraFactoryBase: Unknown generator ID = " << generatorID << std::endl;
    return false;
  }
}

}
}