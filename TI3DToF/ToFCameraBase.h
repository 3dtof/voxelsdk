/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCAMERABASE_H
#define VOXEL_TI_TOFCAMERABASE_H

#include <DepthCamera.h>

#include "TI3DToFExports.h"
#include "ToFDepthFrameGenerator.h"

namespace Voxel
{
  
namespace TI
{

/// This is a generic ToFCamera which is not strictly dependent on TI's ToF chips
class TI3DTOF_EXPORT ToFCameraBase : public DepthCamera
{
protected:
  RawDataFramePtr _rawDataFrame; // Used by _captureDepthFrame(). This is not exposed to DepthCamera
  virtual bool _captureRawUnprocessedFrame(RawFramePtr &rawFrame);
  virtual bool _convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame);
  
  virtual bool _start();
  virtual bool _stop();
  
  virtual bool _initStartParams();
  
  virtual bool _getAmplitudeNormalizingFactor(float &factor) = 0;
  virtual bool _getDepthScalingFactor(float &factor) = 0;
  
  bool _init();
  virtual bool _onReset();
  
  Ptr<ToFDepthFrameGenerator> _tofDepthFrameGenerator;
  
public:
  ToFCameraBase(const String &name, const String &chipset, DevicePtr device): DepthCamera(name, chipset, device), 
    _tofDepthFrameGenerator(new ToFDepthFrameGenerator()) 
    {
      _frameGenerators[1] = std::dynamic_pointer_cast<FrameGenerator>(_tofDepthFrameGenerator);
    }
  
  virtual ~ToFCameraBase() {}
};

}
}

#endif // TOFCAMERABASE_H
