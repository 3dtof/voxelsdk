/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCALCULUSCAMERA_H
#define VOXEL_TI_TOFCALCULUSCAMERA_H

#include <ToFCamera.h>

#define MOD_M_FRAC "mod_m_frac"
#undef MOD_PS1
#define MOD_PS "mod_ps"
#undef MOD_PS2
#undef MOD_FREQ1
#undef MOD_FREQ2
#define MOD_F "mod_freq1"

#undef DEALIAS_16BIT_OP_ENABLE
#undef DEALIASED_PHASE_MASK

#define ALT_FRM_EN "alt_frm_en"
#define ALT_FREQ_SEL "alt_freq_sel"
#define SUP_FRM_INTG_SCALE "sup_frm_intg_scale"

#define STANDBY "standby"
#define DIS_SDMOD "dis_sdmod"
#define INIT_0 "init_0"
#define INIT_1 "init_1"
#define INIT_2 "init_2"
#define INIT_3 "init_3"
#define INIT_4 "init_4"
#define INIT_5 "init_5"
#define INIT_6 "init_6"
#define INIT_7 "init_7"
#define INIT_8 "init_8"
#define UPDATE_SEL "update_sel"
#define OP_DATA_ARRANGE_MODE "op_data_arrange_mode"
#define MOD_CDRIV_EN "mod_cdriv_en"
#define MOD_CDRIV_CURR "mod_cdriv_curr"
#define SHUTTER_EN "shutter_en"
#define LUMPED_DEAD_TIME "lumped_dead_time"

#undef CONFIDENCE_THRESHOLD
#define CONFIDENCE_THRESHOLD "amplitude_threshold"

#undef SUBFRAME_CNT_MAX
#define SUBFRAME_CNT_MAX1 "sub_frame_cnt_max1"
#define SUBFRAME_CNT_MAX2 "sub_frame_cnt_max2"

#define COEFF_ILLUM "coeff_illum"
#define COEFF_SENSOR "coeff_sensor"

#define NONLINEARITY_PHASE_PERIOD "phase_lin_corr_period"
#define NONLINEARITY_COEFF "phase_lin_coeff"
#define NONLINEARITY_DISABLE "phase_lin_corr_dis"

// For unambigous range
#define SCRATCH2 "scratch2"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT ToFCalculusCamera: public ToFCamera
{
protected:
  bool _init();
  
  virtual bool _initStartParams();
  
  virtual bool _getIlluminationFrequency(float& frequency) const;
  bool _getMaximumFrameSize(FrameSize &s) const;
  
  virtual bool _getFrameSize(Voxel::FrameSize &s) const;
  virtual bool _getBytesPerPixel(uint &bpp) const;
  virtual bool _getOpDataArrangeMode(int &dataArrangeMode) const;
  virtual bool _getSystemClockFrequency(uint &frequency) const;
  
  virtual bool _getToFFrameType(ToFFrameType &frameType) const;
  virtual bool _allowedROI(String &message);
  virtual bool _getROI(RegionOfInterest &roi) const;
  virtual bool _setROI(const RegionOfInterest &roi);
  virtual bool _getBinning(uint &rowsToMerge, uint &columnsToMerge) const;
  virtual bool _setBinning(uint rowsToMerge, uint columnsToMerge, const FrameSize &frameSize);
  
  virtual bool _is16BitModeEnabled(bool &mode16Bit);
  virtual bool _getDealiasedPhaseMask(int &dealiasedPhaseMask);
  
  virtual bool _getSubFrameCount(int &subframeCount) const;
  
  virtual bool _isHistogramEnabled() const;
  
  virtual bool _applyCalibrationParams();

  bool _updateEasyConf();
  
  virtual bool _getCurrentProfileRegisterName(String& name);
    
public:
  ToFCalculusCamera(const String &name, DevicePtr device);
  
  
  
  virtual ~ToFCalculusCamera() {}
  
  friend class CalculusVCOFrequency;
  friend class CalculusModulationFrequencyParameter;
  friend class CalculusIntegDutyCycle;
  friend class CalculusUnambiguousRangeParameter;
};

class TI3DTOF_EXPORT CalculusModulationFrequencyParameter: public FloatParameter
{
  ToFCalculusCamera &_depthCamera;
  float _optimalMinimum, _optimalMaximum;
public:
  CalculusModulationFrequencyParameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer, const float &optimalMinimum, const float &optimalMaximum, const float &defaultValue):
  FloatParameter(programmer, MOD_F, "MHz", 0, 0, 0, 1, 2.5f, 600.0f, defaultValue, "Modulation frequency", "Frequency used for modulation of illumination", 
                 Parameter::IO_READ_WRITE, {VCO_FREQ, MOD_M, MOD_M_FRAC, MOD_N, MOD_PS}), _optimalMinimum(optimalMinimum), _optimalMaximum(optimalMaximum), _depthCamera(depthCamera) {}
                 
  const float getOptimalMaximum() { return _optimalMaximum; }
  const float getOptimalMinimum() { return _optimalMinimum; }
                 
  virtual const float lowerLimit() const;
  virtual const float upperLimit() const;
                 
  virtual bool get(float &value, bool refresh = false);
  
  virtual bool set(const float &value);
  
  virtual ~CalculusModulationFrequencyParameter() {}
};

class TI3DTOF_EXPORT CalculusUnambiguousRangeParameter: public UnsignedIntegerParameter
{
  ToFCalculusCamera &_depthCamera;
  uint _defaultValue;
public:
  CalculusUnambiguousRangeParameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer, const uint &minimum, const uint &maximum, const uint &defaultValue):
    UnsignedIntegerParameter(programmer, UNAMBIGUOUS_RANGE, "m", 0, 0, 23, 16, minimum, maximum, 
                           defaultValue, "Unambiguous Range", "Unambiguous range of distance the camera needs to support"), _defaultValue(defaultValue),
                           _depthCamera(depthCamera) {} //FIXME 0x5839, 24 -- this register is not working
                           
  virtual bool get(uint &value, bool refresh = false);
  
  virtual bool set(const uint &value);
  
  virtual ~CalculusUnambiguousRangeParameter() {}
};

}
}

#endif // VOXEL_TI_TOFCALCULUSCAMERA_H
