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
  String _vcoFreq, _modM, _modMFrac, _modN;
public:
  CalculusVCOFrequency(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer, const String &vcoFreq, const String &modM, const String &modMFrac, const String &modN):
  FloatParameter(programmer, VCO_FREQ, "MHz", 0, 0, 0, 1, 300, 600, 384, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, {modM, modN}), _depthCamera(depthCamera),
                  _vcoFreq(vcoFreq), _modM(modM), _modMFrac(modMFrac), _modN(modN) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    uint modM, modMFrac, modN, systemClockFrequency;
    if (!_depthCamera._get(_modM, modM, refresh) || !_depthCamera._get(_modMFrac, modMFrac, refresh)
        || !_depthCamera._get(_modN, modN, refresh) || !_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;

    float modMf = modM + ((float) modMFrac)/(1 << 16);

    float v = 2 * systemClockFrequency * modMf/(1 << modN);

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
    
    float modMf = value * modN / systemClockFrequency;
    modM = (uint) modMf;
    modMFrac = (uint)(modMf - modM) * (1 << 16);
    
    if(!_depthCamera._set(_modM, modM) || !_depthCamera._set(_modN, modN) || !_depthCamera._set(_modMFrac, modMFrac))
      return false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    _value = value;
    
    return true;
  }
  
  virtual ~CalculusVCOFrequency() {}
};

class CalculusModulationFrequencyParameter: public FloatParameter
{
  ToFCalculusCamera &_depthCamera;
  String _modPS, _vcoFreq;
public:
  CalculusModulationFrequencyParameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer, const String &name, const String &vcoFreq, const String &modPS):
  FloatParameter(programmer, name, "MHz", 0, 0, 0, 1, 2.5f, 600.0f, 48, "Modulation frequency", "Frequency used for modulation of illumination", 
                 Parameter::IO_READ_WRITE, {vcoFreq, modPS}), _vcoFreq(vcoFreq), _modPS(modPS), _depthCamera(depthCamera) {}
                 
  virtual const float lowerLimit() const
  {
    int quadCount = 4;

    //if (!_depthCamera._get(QUAD_CNT_MAX, quadCount))
    //  return _lowerLimit;

    if (quadCount == 0)
      return _lowerLimit;

    return 37.5f/quadCount;
  }

  virtual bool get(float &value, bool refresh = false)
  {
    float vcoFrequency;
    
    uint modulationPS;
    int quadCount = 4;

    if (!_depthCamera._get(_vcoFreq, vcoFrequency, refresh) || !_depthCamera._get(_modPS, modulationPS, refresh))
      return false;

    if (quadCount == 0)
      return false;

    float v = vcoFrequency/quadCount/(1 + modulationPS);

    if (!validate(v))
      return false;

    value = v;
    return true;
    
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    ParameterPtr p = _depthCamera.getParam(_vcoFreq);

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
    
    if(!_depthCamera._set(_modPS, modulationPS))
      return false;
    
    float val;
    
    if(!v.get(val))
      return false;
    
    return true;
  }
};

#define DEFAULT_UNAMBIGUOUS_RANGE 4095*SPEED_OF_LIGHT/1E6F/2/24/(1 << 12)
class CalculusUnambiguousRangeParameter: public UnsignedIntegerParameter
{
  ToFCalculusCamera &_depthCamera;
public:
  CalculusUnambiguousRangeParameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, UNAMBIGUOUS_RANGE, "m", 0, 0, 0, 0, 3, 50, DEFAULT_UNAMBIGUOUS_RANGE, "Unambiguous Range", "Unambiguous range of distance the camera needs to support"), _depthCamera(depthCamera) {}

  virtual bool get(uint &value, bool refresh = false)
  {
    _value = DEFAULT_UNAMBIGUOUS_RANGE;
    value = DEFAULT_UNAMBIGUOUS_RANGE;
    return true;
  }

  virtual bool set(const uint &value)
  {
    return true;
  }

};

class CalculusPhaseCorr1Parameter: public IntegerParameter
{
protected:
  ToFCalculusCamera &_depthCamera;

public:
  CalculusPhaseCorr1Parameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  IntegerParameter(programmer, PHASE_CORR_1, "", 0, 0, 0, 0, -4096, 4095, 0, "", "Phase correction for base frequency. This value is subtracted from the obtained phase.", Parameter::IO_READ_WRITE, {}), _depthCamera(depthCamera)
  {}

  virtual bool get(int &value, bool refresh = false)
  {
    value = -1 * _value;
    return true;
  }

  virtual bool set(const int &value)
  {
    if (!validate(value))
      return false;

    _value = -1 *value;
    return (_depthCamera._set(CALIB_PHASE_OFFSET, _value));
  }
};

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

bool ToFCalculusCamera::_getFrameRate(FrameRate &r) const
{
  r.numerator = 30;
  r.denominator = 1;
  return true;
}

bool ToFCalculusCamera::_setFrameRate(const FrameRate &r)
{
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
    ParameterPtr(new CalculusVCOFrequency(*this, *_programmer, VCO_FREQ, MOD_M1, MOD_M_FRAC1, MOD_N1)),
    ParameterPtr(new CalculusModulationFrequencyParameter(*this, *_programmer, MOD_FREQ1, VCO_FREQ, MOD_PS1)),
    ParameterPtr(new CalculusUnambiguousRangeParameter(*this, *_programmer
      )),
  }))
    return false;
  
  /* Replace phase_corr_1 (and later _2 with its inverse parameter */
  _parameters.erase(PHASE_CORR_1);
  if (!_addParameters({
    ParameterPtr(new CalculusPhaseCorr1Parameter(*this, *_programmer)),
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
      !set(UPDATE_SEL, 2) ||
      !set(SHUTTER_EN, true) ||
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
  
  if(!get(MOD_FREQ1, frequency))
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

 
}
}