/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXEL14_CAMERAFACTORY_H
#define VOXEL_TI_VOXEL14_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class Voxel14CameraFactory: public ToFCameraFactoryBase
{
public:
  Voxel14CameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~Voxel14CameraFactory() {}
};
  
}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
