/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFHaddockCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

#undef K0
#define K0 "k0"

namespace Voxel
{
  
namespace TI
{
  
/// Custom parameters
class HaddockVCOFrequency: public FloatParameter
{
  ToFHaddockCamera &_depthCamera;
public:
  HaddockVCOFrequency(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, VCO_FREQ, "MHz", 0, 0, 0, 1, 600, 1200, 864, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, {MOD_M, MOD_N}), _depthCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    uint modM, modN, systemClockFrequency;
    if(!_depthCamera._get(MOD_M, modM, refresh) || !_depthCamera._get(MOD_N, modN, refresh) || !_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;
    
    if(modN == 0)
      return false;
    
    float v = systemClockFrequency*modM/modN;
    
    if(!validate(v))
      return false;
    
    value = v;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    uint modM, modN, systemClockFrequency;
    if(!_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;
    
    modN = 18;
    
    if(systemClockFrequency == 0)
      return false;
    
    modM = value*modN/2/systemClockFrequency;
    modM *= 2;  // mod_m lsbit has no effect and should be set to 0
    
    if(!_depthCamera._set(MOD_N, modN))
      return false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    if(!_depthCamera._set(MOD_M, modM))
      return false;
    
    _value = modM*systemClockFrequency/modN;
    
    return true;
  }
  
  virtual ~HaddockVCOFrequency() {}
};

class HaddockModulationFrequencyParameter: public FloatParameter
{
  ToFHaddockCamera &_depthCamera;
  String _psName;
public:
  HaddockModulationFrequencyParameter(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer, const String &name, const String &psName):
  FloatParameter(programmer, name, "MHz", 0, 0, 0, 1, 6.25f, 433.333f, 18, "Modulation frequency", "Frequency used for modulation of illumination", 
                 Parameter::IO_READ_WRITE, {psName}), _psName(psName), _depthCamera(depthCamera) {}
                 
  virtual const float getOptimalMaximum() const { return 24; }
  virtual const float getOptimalMinimum() const { return 12; }
                 
  virtual bool get(float &value, bool refresh = false)
  {
    float vcoFrequency;
    
    uint modulationPS;
    
    if(!_depthCamera._get(VCO_FREQ, vcoFrequency, refresh) || !_depthCamera._get(_psName, modulationPS, refresh))
      return false;
    
    float v = vcoFrequency/3/modulationPS;
    
    if(!validate(v))
      return false;
    
    value = v;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    ParameterPtr p = _depthCamera.getParam(VCO_FREQ),
    p2 = _depthCamera.getParam(_psName);
    
    if(!p || !p2)
      return false;
    
    HaddockVCOFrequency &v = (HaddockVCOFrequency &)*p;
    UnsignedIntegerParameter &modPS = (UnsignedIntegerParameter &)*p2;
    
    uint modulationPS = v.lowerLimit()/3/value + 1;
    
    float vcoFrequency;
    
    if(modulationPS > modPS.upperLimit())
    {
      vcoFrequency = modPS.upperLimit()*3*value;
      modulationPS = modPS.upperLimit();
    }
    else
      vcoFrequency = modulationPS*3*value;
    
    if(!_depthCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) 
    { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    if(!_depthCamera._set(_psName, modulationPS))
      return false;
    
    if(!v.set(modulationPS*3*value))
      return false;
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    float val;
    
    if(!v.get(val))
      return false;
    
    return true;
  }
  
  virtual ~HaddockModulationFrequencyParameter() {}
};

// NOTE: 0x5C60 is a software_state register available for software to store bits in Haddock camera.
#define DEFAULT_UNAMBIGUOUS_RANGE 4095*SPEED_OF_LIGHT/1E6f/2/18/(1 << 12)
class HaddockUnambiguousRangeParameter: public UnsignedIntegerParameter
{
  ToFHaddockCamera &_depthCamera;
public:
  HaddockUnambiguousRangeParameter(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, UNAMBIGUOUS_RANGE, "m", 0x5C60, 24, 5, 0, 3, 60, 
                           DEFAULT_UNAMBIGUOUS_RANGE, "Unambiguous Range", "Unambiguous range of distance the camera needs to support"),
                           _depthCamera(depthCamera) {}
                           
  virtual bool get(uint &value, bool refresh = false)
  {
    if(refresh)
    {
      if(!UnsignedIntegerParameter::get(value, refresh))// Invalid value? Probably register not set yet
      {
        if(!set(DEFAULT_UNAMBIGUOUS_RANGE)) // set range of 3 meters
          return false;
        
        _value = DEFAULT_UNAMBIGUOUS_RANGE;
        value = DEFAULT_UNAMBIGUOUS_RANGE;
      }
      return true;
    }
    else
      return UnsignedIntegerParameter::get(value, refresh);
  }
  
  virtual bool set(const uint &value)
  {
    ParameterPtr p = _depthCamera.getParam(MOD_FREQ1);
    
    HaddockModulationFrequencyParameter *mfp = dynamic_cast<HaddockModulationFrequencyParameter *>(p.get());
    
    HaddockVCOFrequency *vco = dynamic_cast<HaddockVCOFrequency *>(_depthCamera.getParam(VCO_FREQ).get());
    
    if(!mfp || !vco)
      return false;
    
    float modulationFrequency1Minimum = mfp->getOptimalMinimum();
    
    FrameRate r;
    
    if(!_depthCamera._getFrameRate(r))
      return false;
    
    // No need of dealiasing?
    if(value <= (uint)(4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/modulationFrequency1Minimum))
    {
      if(!_depthCamera._set(TG_EN, false) || 
        !_depthCamera._set(QUAD_CNT_MAX, 4) ||
        !_depthCamera._setFrameRate(r) ||
        !_depthCamera._set(DEALIAS_16BIT_OP_ENABLE, false) ||
        !_depthCamera._set(DEALIASED_PHASE_MASK, 0))
        return false;
      
      if(!mfp->set(4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/value))
        return false;
      
      if(!_depthCamera._set(DEALIAS_EN, false)) // Disable dealiasing explicitly
        return false;
      
      if(!UnsignedIntegerParameter::set(value)) // Save the value in a register
        return false;
      
      if(!_depthCamera._set(TG_EN, true))
        return false;
      
      return true;
    }
    else
    {
      if(!_depthCamera._set(TG_EN, false) ||
        !_depthCamera._set(QUAD_CNT_MAX, 6))
        return false;
      
      uint ma = 8, mb = 7, k0 = 1, modPS1 = 14, modPS2 = 16;
      
      float vcoFreqMinimum = std::max(mfp->getOptimalMinimum()*3*modPS2, vco->lowerLimit()), 
      vcoFreqMaximum = std::min(mfp->getOptimalMaximum()*3*modPS2, vco->upperLimit());
      
      float s = 96*SPEED_OF_LIGHT*1E-6/value; // in MHz
      
      int phaseMask = 0;
      int sign = 1;
      
      float vcoFreq;
      
      if(s > vcoFreqMaximum)
      {
        sign = 1;
        while(s/(1 << phaseMask) > vcoFreqMaximum)
          phaseMask++;
        
        vcoFreq = s/(1 << phaseMask);
        
        if(vcoFreq < vcoFreqMinimum)
        {
          phaseMask--;
          vcoFreq = s/(1 << phaseMask);
        }
      }
      else if(s < vcoFreqMinimum)
      {
        sign = -1;
        while(s*(1 << phaseMask) < vcoFreqMinimum)
          phaseMask++;
        
        vcoFreq = s*(1 << phaseMask);
      }
      else
        vcoFreq = s;
      
      if(!_depthCamera._set(MOD_PLL_UPDATE, true))
        return false;
      
      ParameterPtr pllUpdate(nullptr, [this](Parameter *) 
      { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
      
      if(!_depthCamera._set(VCO_FREQ, vcoFreq) ||
        !_depthCamera._set(MOD_PS1, modPS1) ||
        !_depthCamera._set(MOD_PS2, modPS2) ||
        !_depthCamera._set(MA, ma) || 
        !_depthCamera._set(MB, mb) || 
        !_depthCamera._set(K0, k0) || 
        !_depthCamera._set(DEALIASED_PHASE_MASK, (int)sign*phaseMask) ||
        !_depthCamera._setFrameRate(r) ||
        !_depthCamera._set(DEALIAS_16BIT_OP_ENABLE, false) ||
        !_depthCamera._set(DEALIAS_EN, true)) // Enable dealiasing explicitly
        return false;
      
      if(!UnsignedIntegerParameter::set(value)) // Save the value in a register
        return false;
      
      if(!_depthCamera._set(TG_EN, true))
        return false;
      
      return true;
    }
  }
};



class HaddockSensorTemperatureParameter: public IntegerParameter
{
  ToFHaddockCamera &_depthCamera;
public:
  HaddockSensorTemperatureParameter(ToFHaddockCamera &depthCamera, RegisterProgrammer& programmer):
  IntegerParameter(programmer, "tsensor", "", 0, 0, 0, 0, -128, 127, 30, "Sensor Temperature", 
                    "Sensor temperature"), _depthCamera(depthCamera) {}
                    
  virtual bool get(int& value, bool refresh = false)
  {
    return _depthCamera._get("temp_out2", value, refresh);
  }
  
  virtual bool set(const int& value)
  {
    return _depthCamera._set("temp_out2", value);
  }
};

ToFHaddockCamera::ToFHaddockCamera(const String &name, DevicePtr device): ToFCamera(name, "haddock.ti", device)
{
}

bool ToFHaddockCamera::_getSystemClockFrequency(uint &frequency) const
{
  frequency = 48;
  return true;
}


bool ToFHaddockCamera::_getMaximumFrameSize(FrameSize &s) const
{
  s.width = 320;
  s.height = 240;
  return true;
}

  
bool ToFHaddockCamera::_init()
{
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_TEMPERATURE];
    tVector<String> params = {"coeff_illum_1", "coeff_sensor_1", "coeff_illum_2", "coeff_sensor_2"};
    calibInfo.calibrationParameters.insert(calibInfo.calibrationParameters.end(), params.begin(), params.end());
  }
  
  Configuration c;
  
  String name = configFile.get("core", "dml");
  
  if(!name.size() || !c.getConfFile(name)) // true => name is now a proper path
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Failed to locate/read DML file '" << name << "'" << std::endl;
    return false;
  }
  
  ParameterDMLParser p(*_programmer, name);
  
  tVector<ParameterPtr> params;
  
  if(!p.getParameters(params))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not read parameters from DML file '" << name << "'" << std::endl;
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
    ParameterPtr(new PhaseCorrectionAdditiveParameter(false, *_programmer)),
    ParameterPtr(new HaddockVCOFrequency(*this, *_programmer)),
    ParameterPtr(new HaddockModulationFrequencyParameter(*this, *_programmer, MOD_FREQ1, MOD_PS1)),
    ParameterPtr(new HaddockModulationFrequencyParameter(*this, *_programmer, MOD_FREQ2, MOD_PS2)),
    ParameterPtr(new HaddockUnambiguousRangeParameter(*this, *_programmer)),
    ParameterPtr(new HaddockSensorTemperatureParameter(*this, *_programmer))
  }))
    return false;
  
  if(!ToFCamera::_init())
  {
    return false;
  }
  
  return true;
}

bool ToFHaddockCamera::_applyCalibrationParams()
{
  bool commonPhaseCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_COMMON_PHASE_OFFSET_ID)) == 0;
  bool temperatureCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_TEMPERATURE_ID)) == 0;
  
  if(commonPhaseCalibEnable)
  {
    if(!set(PHASE_CORR_1, configFile.getInteger("calib", PHASE_CORR_1)) ||
    !set(PHASE_CORR_2, configFile.getInteger("calib", PHASE_CORR_2)) ||
    !set(TILLUM_CALIB, (uint)configFile.getInteger("calib", TILLUM_CALIB)) ||
    !set(TSENSOR_CALIB, (uint)configFile.getInteger("calib", TSENSOR_CALIB)) ||
    !set(DISABLE_OFFSET_CORR, configFile.getBoolean("calib", DISABLE_OFFSET_CORR)))
      return false;
  }
  else
  {
    if(!set(DISABLE_OFFSET_CORR, true) ||
      !set(PHASE_CORR_1, 0) ||
      !set(PHASE_CORR_2, 0)
    )
      return false;
  }
  
  if(temperatureCalibEnable)
  {
    if(!set(DISABLE_TEMP_CORR, configFile.getBoolean("calib", DISABLE_TEMP_CORR)))
      return false;
  
    int factor = 1;
    
    if(configFile.isPresent("calib", CALIB_PREC))
      factor = (configFile.getInteger("calib", CALIB_PREC) == 0)?1:16;
    
    return
    set(COEFF_ILLUM_1, configFile.getInteger("calib", COEFF_ILLUM_1)/factor) &&
    set(COEFF_ILLUM_2, configFile.getInteger("calib", COEFF_ILLUM_2)/factor) &&
    set(COEFF_SENSOR_1, configFile.getInteger("calib", COEFF_SENSOR_1)/factor) &&
    set(COEFF_SENSOR_2, configFile.getInteger("calib", COEFF_SENSOR_2)/factor);
  }
  else
  {
    if(!set(DISABLE_TEMP_CORR, true))
      return false;
  }
  
  return true;
}


bool ToFHaddockCamera::_initStartParams()
{
  if(!set(TG_EN, true) || 
     !set(BLK_SIZE, 1024U) ||
     !set(BLK_HEADER_EN, true) ||
     !set(OP_CS_POL, true) ||
     !set(FB_READY_EN, true) ||
     !set(CONFIDENCE_THRESHOLD, 1U) ||
     //!set(DEBUG_EN, true) || // Uncomment this for sample data
     !set(ILLUM_EN_POL, false))
    // || set(INTG_TIME, 40.0f);
    return false;
  
  return ToFCamera::_initStartParams();
}

bool ToFHaddockCamera::_allowedROI(String &message)
{
  message  = "Column start and end must be multiples of 16, both between 0 to 319. ";
  message += "Row start and end must be between 0 to 239.";
  return true;
}

bool ToFHaddockCamera::_getROI(RegionOfInterest &roi) const
{
  uint rowStart, rowEnd, colStart, colEnd;
  
  if(!_get(ROW_START, rowStart) || !_get(ROW_END, rowEnd) || !_get(COL_START, colStart) || !_get(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  colStart = colStart*2;
  colEnd = (colEnd + 1)*2 - 1;
  
  roi.x = colStart;
  roi.y = rowStart;
  roi.width = colEnd - colStart + 1;
  roi.height = rowEnd - rowStart + 1;
  return true;
}

bool ToFHaddockCamera::_setROI(const RegionOfInterest &roi)
{
  if(isRunning())
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Cannot set frame size while the camera is streaming" << std::endl;
    return false;
  }
  
  uint rowStart, rowEnd, colStart, colEnd;
  
  colStart = (roi.x/16)*8;
  colEnd = ((roi.x + roi.width)/16)*8 - 1;
  
  rowStart = roi.y;
  rowEnd = rowStart + roi.height - 1;
  
  if(!_set(ROW_START, rowStart) || !_set(ROW_END, rowEnd) || !_set(COL_START, colStart) || !_set(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  if(!_setBinning(1, 1, roi))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not reset binning while setting ROI." << std::endl;
    return false;
  }
  
  FrameSize s;
  
  if(!_getFrameSize(s) || !_setFrameSize(s, false)) // Get and set frame size to closest possible
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not update frame size to closest valid frame size" << std::endl;
    return false;
  }
  
  return true;
}

bool ToFHaddockCamera::_isHistogramEnabled() const
{
  bool histogramEnabled;
  return _get(HISTOGRAM_EN, histogramEnabled) && histogramEnabled;
}

bool ToFHaddockCamera::_getIlluminationFrequency(float& frequency) const
{
  float modulationFrequency1;
  bool dealiasingEnabled;
  
  uint modulationPS1, modulationPS2;
  
  if(!get(DEALIAS_EN, dealiasingEnabled))
    return false;
  
  if(dealiasingEnabled)
  {
    if(!get(MOD_FREQ1, modulationFrequency1)) // ensure that these are valid
      return false;
    
    if(!get(MOD_PS1, modulationPS1) || !get(MOD_PS2, modulationPS2))
      return false;
    
    frequency = modulationFrequency1*gcd(modulationPS1, modulationPS2)/modulationPS2;
    
    return true;
  }
  else
  {
    if(!get(MOD_FREQ1, frequency))
      return false;
    
    return true;
  }
}

bool ToFHaddockCamera::_getCurrentProfileRegisterName(String &name)
{
  name = CURRENT_PROFILE;
  return true;
}
 
}
}
