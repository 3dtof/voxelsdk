/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCAMERAFACTORY_H
#define VOXEL_TI_TOFCAMERAFACTORY_H

#include <DepthCameraFactory.h>

#include "TI3DToFExports.h"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFCameraFactoryBase: public DepthCameraFactory
{
public:
  ToFCameraFactoryBase(const String &name);
  
  virtual bool getChannels(Device &device, Vector<int> &channels);
  
  virtual Vector<GeneratorIDType> getSupportedGeneratorTypes();
  virtual bool getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator);
  
  virtual ~ToFCameraFactoryBase() {}
};

}
}

#endif // VOXEL_TI_TOFCAMERAFACTORY_H
