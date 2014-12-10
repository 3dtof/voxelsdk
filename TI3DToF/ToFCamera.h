/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCAMERA_H
#define VOXEL_TI_TOFCAMERA_H

#include <DepthCamera.h>
#include <RegisterProgrammer.h>
#include <Streamer.h>

namespace Voxel
{
  
namespace TI
{

class ToFCamera: public DepthCamera
{
protected:
  Ptr<RegisterProgrammer> _programmer;
  Ptr<Streamer> _streamer;
  
public:
  ToFCamera(DevicePtr device): DepthCamera(device) {}
  
  virtual bool isInitialized()
  {
    return _programmer and _programmer->isInitialized() and 
            _streamer and _streamer->isInitialized();
  }
};

}
}

#endif // TOFCAMERA_H
