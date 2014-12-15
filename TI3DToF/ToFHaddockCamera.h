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
  
  virtual bool _processRawFrame(RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput); // here output raw frame will have processed data, like ToF data for ToF cameras
  virtual bool _convertToDepthFrame(RawFramePtr &rawFrame, DepthFramePtr &depthFrame);
  
public:
  ToFHaddockCamera(const String &name, DevicePtr device);
  
  virtual bool setFrameRate(const FrameRate &r);
  virtual bool getFrameRate(FrameRate &r);
  
  virtual bool getFrameSize(FrameSize &s);
  virtual bool setFrameSize(const FrameSize &s);
  
  virtual ~ToFHaddockCamera() {}
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
