/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_CALCULUS_CDK_CAMERAFACTORY_H
#define VOXEL_TI_CALCULUS_CDK_CAMERAFACTORY_H

#include <DepthCameraFactory.h>
#include <ToFCameraFactoryBase.h>

namespace Voxel
{
  
namespace TI
{

class CalculusCDKCameraFactory: public ToFCameraFactoryBase
{
public:
  CalculusCDKCameraFactory(const String &name);
  
  virtual DepthCameraPtr getDepthCamera(DevicePtr device);
  
  virtual ~CalculusCDKCameraFactory() {}
};

}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
