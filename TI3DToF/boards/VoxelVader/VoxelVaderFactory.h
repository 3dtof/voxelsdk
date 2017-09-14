/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VoxelVader_CAMERAFACTORY_H
#define VOXEL_TI_VoxelVader_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class VoxelVaderFactory: public ToFCameraFactoryBase
{
public:
	VoxelVaderFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~VoxelVaderFactory() {}
};
  
}
}

#endif // VOXEL_TI_VoxelVader_CAMERAFACTORY_H
