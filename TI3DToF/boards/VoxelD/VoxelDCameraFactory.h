/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXELD_CAMERAFACTORY_H
#define VOXEL_TI_VOXELD_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class VoxelDCameraFactory: public ToFCameraFactoryBase
{
public:
  VoxelDCameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~VoxelDCameraFactory() {}
};
  
}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
