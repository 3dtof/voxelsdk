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

class TI3DTOF_EXPORT ToFHaddockCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _getDepthScalingFactor(float &factor);
  bool _getMaximumFrameSize(FrameSize &s) const;
  
  virtual bool _getSystemClockFrequency(uint &frequency) const;
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi);
  virtual bool _setROI(const RegionOfInterest &roi);
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();
    
public:
  ToFHaddockCamera(const String &name, DevicePtr device);
  
  
  
  virtual ~ToFHaddockCamera() {}
  
  friend class HaddockVCOFrequency;
  friend class HaddockModulationFrequencyParameter;
  friend class HaddockUnambiguousRangeParameter;
  friend class HaddockSensorTemperatureParameter;
};

}
}

#endif // VOXEL_TI_TOFHADDOCKCAMERA_H
