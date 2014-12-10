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
  DevicePtr _device, _controlDevice, _streamDevice;
  
public:
  DepthCamera(DevicePtr device, DevicePtr controlDevice, DevicePtr streamDevice): _device(device), _controlDevice(controlDevice), _streamDevice(streamDevice) {}
  
  virtual bool isInitialized() = 0;
};


typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
