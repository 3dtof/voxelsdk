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
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
public:
  ToFHaddockCamera(const String &name, DevicePtr device);
  
  virtual bool setFrameRate(const FrameRate &r);
  virtual bool getFrameRate(FrameRate &r);
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
