/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFHADDOCKCAMERA_H
#define VOXEL_TI_TOFHADDOCKCAMERA_H

#include <ToFCamera.h>

#define CURRENT_PROFILE "scratch2" //sys_clk_freq used as scratch register

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFHaddockCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  bool _getMaximumFrameSize(FrameSize &s) const;
  
  virtual bool _getSystemClockFrequency(uint &frequency) const;
  virtual bool _getIlluminationFrequency(float& frequency) const;
  
  
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi) const;
  virtual bool _setROI(const RegionOfInterest &roi);
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();
  
  virtual bool _getCurrentProfileRegisterName(String& name);
    
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
