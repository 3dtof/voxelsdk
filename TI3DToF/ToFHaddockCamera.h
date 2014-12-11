/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFHADDOCKCAMERA_H
#define VOXEL_TI_TOFHADDOCKCAMERA_H

#include <ToFCamera.h>

namespace Voxel
{
  
namespace TI
{

class ToFHaddockCamera: public ToFCamera
{
public:
  ToFHaddockCamera(const String &name, DevicePtr device): ToFCamera(name, device) {}
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
