/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_HADDOCK_CDK_CAMERAFACTORY_H
#define VOXEL_TI_HADDOCK_CDK_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{
  
class HaddockCDKCameraFactory: public ToFCameraFactoryBase
{
public:
  HaddockCDKCameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~HaddockCDKCameraFactory() {}
};
  
}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
