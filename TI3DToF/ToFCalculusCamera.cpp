/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFCalculusCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

namespace Voxel
{
  
namespace TI
{
  
/// Custom parameters

class CalculusVCOFrequency: public FloatParameter
{
  ToFCalculusCamera &_depthCamera;
public:
  CalculusVCOFrequency(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, VCO_FREQ, "MHz", 0, 0, 0, 1, 300, 600, 384, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, 
		 {MOD_M, MOD_M_FRAC, MOD_N}), _depthCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    uint modM, modMFrac, modN, systemClockFrequency;
    if (!_depthCamera._get(MOD_M, modM, refresh) || !_depthCamera._get(MOD_M_FRAC, modMFrac, refresh)
        || !_depthCamera._get(MOD_N, modN, refresh) || !_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;

    float modMf = modM + ((float) modMFrac)/(1 << 16);

    float v = 2*systemClockFrequency*modMf/(1 << modN);

    if (!validate(v))
      return false;

    value = v;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    if(!_depthCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    uint modM, modMFrac, modN, systemClockFrequency;
    if(!_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;
    
    modN = 2;
    
    if(systemClockFrequency == 0)
      return false;
    
    float modMf = value*(1 << modN)/(2*systemClockFrequency);
    
    modM = (uint) modMf;
    modMFrac = (uint)(modMf - modM) * (1 << 16);
    
    if(!_depthCamera._set(MOD_M, modM) || !_depthCamera._set(MOD_N, modN) || !_depthCamera._set(MOD_M_FRAC, modMFrac))
      return false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    _value = value;
    
    return true;
  }
  
  virtual ~CalculusVCOFrequency() {}
};

const float CalculusModulationFrequencyParameter::lowerLimit() const 
{ 
  int quadCount;
  
  ParameterPtr p = _depthCamera.getParam(VCO_FREQ);
  
  ParameterPtr p1 = _depthCamera.getParam(MOD_PS);
  
  if(!p || !p1)
    return _lowerLimit;
  
  FloatParameter *vco = dynamic_cast<FloatParameter *>(p.get());
  UnsignedIntegerParameter *ps = dynamic_cast<UnsignedIntegerParameter *>(p.get());
  
  if(!vco || !ps)
    return _lowerLimit;
  
  float vcoLowerLimit = vco->lowerLimit();
  uint psUpperLimit = ps->upperLimit();
  
  if(!_depthCamera._get(QUAD_CNT_MAX, quadCount))
    return _lowerLimit;
  
  if(quadCount == 0)
    return _lowerLimit;
  
  return vcoLowerLimit/(1 + psUpperLimit)/quadCount;
}

const float CalculusModulationFrequencyParameter::upperLimit() const 
{ 
  int quadCount;
  
  ParameterPtr p = _depthCamera.getParam(VCO_FREQ);
  
  ParameterPtr p1 = _depthCamera.getParam(MOD_PS);
  
  if(!p || !p1)
    return _upperLimit;
  
  FloatParameter *vco = dynamic_cast<FloatParameter *>(p.get());
  UnsignedIntegerParameter *ps = dynamic_cast<UnsignedIntegerParameter *>(p.get());
  
  if(!vco || !ps)
    return _upperLimit;
  
  float vcoUpperLimit = vco->upperLimit();
  uint psLowerLimit = ps->lowerLimit();
  
  if(!_depthCamera._get(QUAD_CNT_MAX, quadCount))
    return _upperLimit;
  
  if(quadCount == 0)
    return _upperLimit;
  
  return vcoUpperLimit/(1 + psLowerLimit)/quadCount;
}

bool CalculusModulationFrequencyParameter::get(float& value, bool refresh)
{
  float vcoFrequency;
  
  uint modulationPS;
  int quadCount = 4;

  if (!_depthCamera._get(VCO_FREQ, vcoFrequency, refresh) || !_depthCamera._get(MOD_PS, modulationPS, refresh))
    return false;

  if (quadCount == 0)
    return false;

  float v = vcoFrequency/quadCount/(1 + modulationPS);

  if (!validate(v))
    return false;

  value = v;
  return true;
  
}

bool CalculusModulationFrequencyParameter::set(const float &value)
{
  if(!validate(value))
    return false;
  
  ParameterPtr p = _depthCamera.getParam(VCO_FREQ);

  int quadCount = 4;
  
  if(!p)
    return false;
  
  if (quadCount == 0)
    return false;

  CalculusVCOFrequency &v = (CalculusVCOFrequency &)*p;
  
  uint modulationPS = v.lowerLimit()/quadCount/value;
  
  if(!v.set((modulationPS + 1) *quadCount*value))
    return false;
  
  if(!_depthCamera._set(MOD_PLL_UPDATE, true))
    return false;
  
  ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
  
  if(!_depthCamera._set(MOD_PS, modulationPS))
    return false;
  
  float val;
  
  if(!v.get(val))
    return false;
  
  return true;
}

bool CalculusUnambiguousRangeParameter::get(uint &value, bool refresh)
{
  if(!UnsignedIntegerParameter::get(value, refresh) || !validate(value))// Invalid value? Probably register not set yet
  {
    if(validate(_value))
    {
      value = _value; // Use saved value
    }
    else
    {
      if(!set(_defaultValue)) // set range to default value
      return false;
    
      _value = _defaultValue;
      value = _defaultValue;
    }
  }
  
  return true;
}

bool CalculusUnambiguousRangeParameter::set(const uint &value)
{
  ParameterPtr p = _depthCamera.getParam(MOD_F);
  
  CalculusModulationFrequencyParameter *mfp = dynamic_cast<CalculusModulationFrequencyParameter *>(p.get());
  
  if(!mfp)
    return false;
  
  float modulationFrequency1Minimum = mfp->getOptimalMinimum(), modulationFrequency1Maximum = mfp->getOptimalMaximum();
  
  FrameRate r;
  
  if(!_depthCamera._getFrameRate(r))
    return false;
  
  // No need of dealiasing?
  if(value <= (uint)(4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/modulationFrequency1Minimum))
  {
    float modulationFrequency1 = 4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/value;
    
    if(!_depthCamera._set(TG_EN, false) || 
      !_depthCamera._setFrameRate(r) ||
      !mfp->set(modulationFrequency1) ||
      !_depthCamera._set(DEALIAS_EN, false) ||// Disable dealiasing explicitly)
      !_depthCamera._set(ALT_FRM_EN, false) ||
      //!UnsignedIntegerParameter::set(value) // Save the value in a register
      !_depthCamera._set(TG_EN, true))
      return false;
      
    _value = value;
    
    return true;
  }
  else
  {
    int rangeExtensionRatio = ceil(log2(value/(SPEED_OF_LIGHT/(2*modulationFrequency1Maximum*1E6))));
    
    if(rangeExtensionRatio > 6)
    {
      logger(LOG_ERROR) << "CalculusUnambiguousRangeParameter: Desired unambiguous range is too large." << std::endl;
      return false;
    }
    
    if(!_depthCamera._set(TG_EN, false) ||
      !_depthCamera._set(ALT_FRM_EN, true) ||
      !mfp->set(modulationFrequency1Maximum) ||
      !_depthCamera._set(ALT_FREQ_SEL, rangeExtensionRatio - 1) ||
      !_depthCamera._setFrameRate(r) ||
      !_depthCamera._set(DEALIAS_EN, true) ||
      //!UnsignedIntegerParameter::set(value) || // Save the value in a register
      !_depthCamera._set(TG_EN, true))
      return false;
    
    _value = value;
    
    return true;
  }
}


ToFCalculusCamera::ToFCalculusCamera(const String &name, DevicePtr device): ToFCamera(name, "calculus.ti", device)
{
}

bool ToFCalculusCamera::_getFrameSize(Voxel::FrameSize &s) const
{
  s.width = 80;
  s.height = 60;
  return true;
}

bool ToFCalculusCamera::_getBytesPerPixel(uint &bpp) const
{
  bpp = (uint) 4;
  return true;
}

bool ToFCalculusCamera::_getOpDataArrangeMode(int &dataArrangeMode) const
{
  dataArrangeMode = 0;
  return true;
}

bool ToFCalculusCamera::_getSystemClockFrequency(uint &frequency) const
{
  frequency = 24;
  return true;
}


bool ToFCalculusCamera::_getMaximumFrameSize(FrameSize &s) const
{
  s.width = 80;
  s.height = 60;
  return true;
}

  
bool ToFCalculusCamera::_init()
{
  Configuration c;
  
  String name = configFile.get("core", "dml");
  
  if(!name.size() || !c.getConfFile(name)) // true => name is now a proper path
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Failed to locate/read DML file '" << name << "'" << std::endl;
    return false;
  }
  
  ParameterDMLParser p(*_programmer, name);
  
  Vector<ParameterPtr> params;
  
  if(!p.getParameters(params))
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Could not read parameters from DML file '" << name << "'" << std::endl;
    return _parameterInit = false;
  }
  
  for(auto &p: params)
  {
    if((p->address() >> 8) == 0) // bankId == 0
      p->setAddress((0x58 << 8) + p->address());
    else if((p->address() >> 8) == 3) // bankId == 3
      p->setAddress((0x5C << 8) + (p->address() & 0xFF));
  }
  
  if(!_addParameters(params))
    return false;
  
  
  if(!_addParameters({
    ParameterPtr(new PhaseCorrectionAdditiveParameter(true, *_programmer)),
    ParameterPtr(new CalculusVCOFrequency(*this, *_programmer)),
  }))
    return false;
  
  if(!ToFCamera::_init())
  {
    return false;
  }

  if (!set(STANDBY, true) ||
      !set(STANDBY, false) ||
      !set(INIT_0, 0xAU) ||
      !set(INIT_1, 0xAU) ||
      !set(INIT_2, true) ||
      !set(INIT_3, true) ||
      !set(INIT_4, 0x0U) ||
      !set(INIT_5, true) ||
      !set(INIT_6, true) ||
      !set(UPDATE_SEL, 2) ||
      !set(SHUTTER_EN, true) ||
      !set(LUMPED_DEAD_TIME, true) ||
      !set(OP_DATA_ARRANGE_MODE, true))
      return false;
      
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!set(TG_EN, true))
    return false;
  
  return true;
}


bool ToFCalculusCamera::_initStartParams()
{
  return ToFCamera::_initStartParams();
}

bool ToFCalculusCamera::_allowedROI(String &message)
{
  message  = "Column start and end must be multiples of 16, both between 0 to 319. ";
  message += "Row start and end must be between 0 to 239.";
  return true;
}

bool ToFCalculusCamera::_getROI(RegionOfInterest &roi)
{
#if 0
  uint rowStart, rowEnd, colStart, colEnd;
  
  if(!_get(ROW_START, rowStart) || !_get(ROW_END, rowEnd) || !_get(COL_START, colStart) || !_get(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  colStart = colStart*2;
  colEnd = (colEnd + 1)*2 - 1;
  
  roi.x = colStart;
  roi.y = rowStart;
  roi.width = colEnd - colStart + 1;
  roi.height = rowEnd - rowStart + 1;
#else
  roi.x = 0;
  roi.y = 0;
  roi.width = 80;
  roi.height = 60;
#endif

  return true;
}

bool ToFCalculusCamera::_setROI(const RegionOfInterest &roi)
{
#if 0
  if(isRunning())
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Cannot set frame size while the camera is streaming" << std::endl;
    return false;
  }
  
  uint rowStart, rowEnd, colStart, colEnd;
  
  colStart = (roi.x/16)*8;
  colEnd = ((roi.x + roi.width)/16)*8 - 1;
  
  rowStart = roi.y;
  rowEnd = rowStart + roi.height - 1;
  
  if(!_set(ROW_START, rowStart) || !_set(ROW_END, rowEnd) || !_set(COL_START, colStart) || !_set(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  if(!_setBinning(1, 1, roi))
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Could not reset binning while setting ROI." << std::endl;
    return false;
  }
  
  FrameSize s;
  if(!_getFrameSize(s) || !_setFrameSize(s, false)) // Get and set frame size to closest possible
  {
    logger(LOG_ERROR) << "ToFCalculusCamera: Could not update frame size to closest valid frame size" << std::endl;
    return false;
  }
#endif
  
  return true;
}

bool ToFCalculusCamera::_getBinning(uint &rowsToMerge, uint &columnsToMerge) const
{
  rowsToMerge = columnsToMerge = 1;
  return true;
}

bool ToFCalculusCamera::_setBinning(uint rowsToMerge, uint columnsToMerge, const FrameSize &frameSize)
{
  return true;
}

bool ToFCalculusCamera::_isHistogramEnabled() const
{
  return false;
}

bool ToFCalculusCamera::_getToFFrameType(ToFFrameType &frameType) const
{
  frameType = ToF_PHASE_AMPLITUDE;
  return true;
}

bool ToFCalculusCamera::_getIlluminationFrequency(float& frequency) const
{
  float modulationFrequency;
  
  if(!get(MOD_F, frequency))
    return false;
    
  return true;
}

bool ToFCalculusCamera::_is16BitModeEnabled(bool &mode16Bit)
{
  mode16Bit = false;
  return true;
}

bool ToFCalculusCamera::_getDealiasedPhaseMask(int &dealiasedPhaseMask)
{
  dealiasedPhaseMask = 0;
  return true;
}

bool ToFCalculusCamera::_applyCalibrationParams()
{
  bool commonPhaseCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_COMMON_PHASE_OFFSET_ID)) == 0;

  if (commonPhaseCalibEnable) {
    if (!set(PHASE_CORR_1, configFile.getInteger("calib", PHASE_CORR_1)))
      return false;
  } else {
    if (!set(DISABLE_OFFSET_CORR, true) ||
      !set(PHASE_CORR_1, 0)
    )
      return false;
  }
  return true;
}

bool ToFCalculusCamera::_getCurrentProfileRegisterName(String &name)
{
  name = "scratch1";
  return true;
}

bool ToFCalculusCamera::_getSubFrameCount(int &subframeCount) const
{
  bool dealiasEnabled;
  if(!_get(DEALIAS_EN, dealiasEnabled))
    return false;
  
  uint subFrameCount1;
  
  if(!dealiasEnabled)
  {
    if(!_get(SUBFRAME_CNT_MAX1, subFrameCount1))
      return false;
    else
    {
      subframeCount = subFrameCount1;
      return true;
    }
  }
  
  uint subframeCount2;
  
  if(!_get(SUBFRAME_CNT_MAX1, subFrameCount1) || !_get(SUBFRAME_CNT_MAX2, subframeCount2))
    return false;
  
  subframeCount = subFrameCount1 + subframeCount2;
  return true;
}


 
}
}