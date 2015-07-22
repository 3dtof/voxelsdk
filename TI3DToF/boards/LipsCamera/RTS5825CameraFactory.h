/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_LIPSCAMERA_CAMERAFACTORY_H
#define VOXEL_TI_LIPSCAMERA_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class LipsCameraFactory: public ToFCameraFactoryBase
{
public:
  LipsCameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~LipsCameraFactory() {}
};
  
}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
