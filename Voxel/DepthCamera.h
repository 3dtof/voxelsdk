/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEPTHCAMERA_H
#define VOXEL_DEPTHCAMERA_H

#include "Device.h"

namespace Voxel
{
  
class DepthCamera
{
protected:
  DevicePtr _device;
  
public:
  DepthCamera(DevicePtr device): _device(device) {}
  
  virtual bool isInitialized() = 0;
};


typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
