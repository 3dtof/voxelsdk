/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_RTS5825CAMERA_CAMERAFACTORY_H
#define VOXEL_TI_RTS5825CAMERA_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class RTS5825CameraFactory: public ToFCameraFactoryBase
{
public:
  RTS5825CameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~RTS5825CameraFactory() {}
};
  
}
}

#endif // VOXEL_TI_RTS5825CAMERA_CAMERAFACTORY_H
