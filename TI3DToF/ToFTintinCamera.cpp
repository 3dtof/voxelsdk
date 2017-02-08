/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFTintinCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

#include <iterator>
#include <algorithm>

namespace Voxel
{
  
namespace TI
{
  
/// Custom parameters
class TintinVCOFrequency: public FloatParameter
{
  ToFTintinCamera &_depthCamera;
  String _vcoFreq, _modM, _modMFrac, _modN;
public:
  TintinVCOFrequency(ToFTintinCamera &depthCamera, RegisterProgrammer &programmer, const String &vcoFreq, const String &modM, const String &modMFrac, const String &modN):
  FloatParameter(programmer, vcoFreq, "MHz", 0, 0, 0, 1, 300, 600, 384, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, {modM, modN}), _depthCamera(depthCamera),
                 _vcoFreq(vcoFreq), _modM(modM), _modMFrac(modMFrac), _modN(modN) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    uint modM, modMFrac, modN, systemClockFrequency;
    if(!_depthCamera._get(_modM, modM, refresh) || !_depthCamera._get(_modMFrac, modMFrac, refresh)
      || !_depthCamera._get(_modN, modN, refresh) || !_depthCamera._getSystemClockFrequency(systemClockFrequency))
      return false;
    
    if(modN == 0)
      return false;
    
    float modMf = modM + ((float)modMFrac)/(1 << 16);
    
    float v = systemClockFrequency*modMf/modN;
    
    if(!validate(v))
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
    
    float modMf = value*modN/systemClockFrequency;
    modM = (uint)modMf;
    modMFrac = (uint)((modMf - modM)*(1 << 16));
    
    if(!_depthCamera._set(_modM, modM) || !_depthCamera._set(_modN, modN) || !_depthCamera._set(_modMFrac, modMFrac))
      return false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    _value = value;
    
    return true;
  }
  
  virtual ~TintinVCOFrequency() {}
};

bool TintinModulationFrequencyParameter::get(float &value, bool refresh)
{
  float vcoFrequency;
  
  uint modulationPS;
  
  int quadCount;
  
  if(!_depthCamera._get(_vcoFreq, vcoFrequency, refresh) || !_depthCamera._get(_modPS, modulationPS, refresh) || !_depthCamera._get(QUAD_CNT_MAX, quadCount, refresh))
    return false;
  
  if(quadCount == 0)
    return false;
  
  float v = vcoFrequency/quadCount/(1 + modulationPS);
  
  if(!validate(v))
    return false;
  
  value = v;
  return true;
}
  
bool TintinModulationFrequencyParameter::set(const float &value)
{
  if(!validate(value))
    return false;
  
  ParameterPtr p = _depthCamera.getParam(_vcoFreq);
  
  int quadCount;
  
  if(!p || !_depthCamera._get(QUAD_CNT_MAX, quadCount))
    return false;
  
  if(quadCount == 0)
    return false;
  
  TintinVCOFrequency &v = (TintinVCOFrequency &)*p;
  
  uint modulationPS = v.lowerLimit()/quadCount/value; // Using lower limit for setting VCO frequency
  
  if(!v.set((modulationPS + 1)*quadCount*value))
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

bool TintinUnambiguousRangeParameter::get(uint &value, bool refresh)
{
  if(!_depthCamera._get(SCRATCH1, value, refresh) || !validate(value))// Invalid value? Probably register not set yet
  {
    if(validate(_value))
    {
      value = _value; // Use saved value
    }
    else
    {
      if(!set(_defaultValue)) // set range of 3 meters
      return false;
    
      _value = _defaultValue;
      value = _defaultValue;
    }
  }
  
  return true;
}

bool TintinUnambiguousRangeParameter::set(const uint &value)
{
  ParameterPtr p = _depthCamera.getParam(MOD_FREQ1);
  
  TintinModulationFrequencyParameter *mfp = dynamic_cast<TintinModulationFrequencyParameter *>(p.get());
  
  TintinVCOFrequency *vco = dynamic_cast<TintinVCOFrequency *>(_depthCamera.getParam(VCO_FREQ1).get());
  
  if(!mfp || !vco)
    return false;
  
  float modulationFrequency1Minimum = mfp->getOptimalMinimum();
  
  FrameRate r;
  
  uint delayFBCoeff1;
  
  if(!_depthCamera._getFrameRate(r))
    return false;
  
  // No need of dealiasing?
  if(value <= (uint)(4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/modulationFrequency1Minimum))
  {
    float modulationFrequency1 = 4095*SPEED_OF_LIGHT/1E6f/2/(1 << 12)/value;
    delayFBCoeff1 = modulationFrequency1*(1 << 10)/24;
    
    if(!_depthCamera._set(TG_DISABLE, true) || 
      !_depthCamera._set(QUAD_CNT_MAX, 4) || 
      !_depthCamera._setFrameRate(r) ||
      !_depthCamera._set(DEALIAS_16BIT_OP_ENABLE, false) ||
      !_depthCamera._set(DEALIASED_PHASE_MASK, 0) || 
      !mfp->set(modulationFrequency1) ||
      !_depthCamera._set(DELAY_FB_COEFF1, delayFBCoeff1) ||
      !_depthCamera._set(DEALIAS_EN, false) || // Disable dealiasing explicitly
      !_depthCamera._set(SCRATCH1, value) || // Save the value in a register
      !_depthCamera._set(TG_DISABLE, false))
      return false;
      
    _value = value;
    
    return true;
  }
  else
  {
    if(!_depthCamera._set(TG_DISABLE, true) ||
      !_depthCamera._set(QUAD_CNT_MAX, 6))
      return false;
    
    uint ma = 2, mb = 3, ka = 2, kb = 1, modPS1 = _modPS1, modPS2 = _modPS2;
    
    uint freqRatio = ma*(1 << 12)/mb;
    
    float vcoFreqMinimum = std::max(mfp->getOptimalMinimum()*6*(1 + modPS1), vco->lowerLimit()), 
    vcoFreqMaximum = std::min((mfp->getOptimalMaximum()*ma)/mb*6*(1 + modPS1), vco->upperLimit());
    
    float s = (96.0f/ma)*(1 + modPS1)*SPEED_OF_LIGHT*1E-6/value; // in MHz
    
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
        vcoFreq = vcoFreqMaximum;
      }
    }
    else if(s < vcoFreqMinimum)
    {
      sign = -1;
      while(s*(1 << phaseMask) < vcoFreqMinimum)
        phaseMask++;
      
      vcoFreq = s*(1 << phaseMask);
      
      if(vcoFreq > vcoFreqMaximum)
        vcoFreq = vcoFreqMaximum;
    }
    else
      vcoFreq = s;
    
    // delayFBCoeff1 = modFreq2*(1 << 10)/24
    float vcoFreq2 = std::max(vco->lowerLimit(), vcoFreq*mb/ma*(1+modPS2)/(1+modPS1));
    
    delayFBCoeff1 = vcoFreq2/6/(1 + modPS2)*(1 << 10)/24;
    
    if(!_depthCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) 
    { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    if(!_depthCamera._set(VCO_FREQ1, vcoFreq) ||
      !_depthCamera._set(VCO_FREQ2, vcoFreq2) ||
      !_depthCamera._set(MOD_PS1, modPS1) ||
      !_depthCamera._set(MOD_PS2, modPS2) ||
      !_depthCamera._set(MA, ma) || 
      !_depthCamera._set(MB, mb) || 
      !_depthCamera._set(KA, ka) ||
      !_depthCamera._set(KB, kb) ||
      !_depthCamera._set(FREQ_RATIO, freqRatio) ||
      !_depthCamera._set(DELAY_FB_COEFF1, delayFBCoeff1) || 
      !_depthCamera._set(DEALIASED_PHASE_MASK, (int)sign*phaseMask) ||
      !_depthCamera._setFrameRate(r) ||
      !_depthCamera._set(DEALIAS_16BIT_OP_ENABLE, false) ||
      !_depthCamera._set(DEALIAS_EN, true)) // Enable dealiasing explicitly
    return false;
    
    if(!_depthCamera._set(SCRATCH1, value)) // Save the value in a register
      return false;
    
    if(!_depthCamera._set(TG_DISABLE, false))
      return false;
    
    _value = value;
    
    return true;
  }
}

ToFTintinCamera::ToFTintinCamera(const String &name, DevicePtr device): ToFCamera(name, "tintin.ti", device)
{
}

bool ToFTintinCamera::_getSystemClockFrequency(uint &frequency) const
{
  frequency = 48;
  return true;
}


bool ToFTintinCamera::_getMaximumFrameSize(FrameSize &s) const
{
  s.width = 320;
  s.height = 240;
  return true;
}

  
bool ToFTintinCamera::_init()
{
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_TEMPERATURE];
    Vector<String> params = {"coeff_illum", "coeff_sensor", CALIB_PREC};
    calibInfo.calibrationParameters.insert(calibInfo.calibrationParameters.end(), params.begin(), params.end());
  }
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_CROSS_TALK];
    calibInfo.name = ToF_CALIB_SECT_CROSS_TALK;
    calibInfo.id = ToF_CALIB_SECT_CROSS_TALK_ID;
    calibInfo.definingParameters = {"unambiguous_range", "mix_volt"};
    calibInfo.calibrationParameters = {"x_filt_coeff_f1", "y_filt_coeff_f1", "x_filt_coeff_f2", "y_filt_coeff_f2"};
  }
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_NON_LINEARITY];
    calibInfo.name = ToF_CALIB_SECT_NON_LINEARITY;
    calibInfo.id = ToF_CALIB_SECT_NON_LINEARITY_ID;
    calibInfo.definingParameters = {"op_clk_freq", "unambiguous_range", "frame_rate", "sub_frame_cnt_max", "quad_cnt_max", "mix_volt", "intg_time"};
    calibInfo.calibrationParameters = {"phase_lin_corr_period", "phase_lin_coeff0", "phase_lin_coeff1"};
  }
  
  Configuration c;
  
  String name = configFile.get("core", "dml");
  
  if(!name.size() || !c.getConfFile(name)) // true => name is now a proper path
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Failed to locate/read DML file '" << name << "'" << std::endl;
    return false;
  }
  
  ParameterDMLParser p(*_programmer, name);
  
  Vector<ParameterPtr> params;
  
  if(!p.getParameters(params))
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Could not read parameters from DML file '" << name << "'" << std::endl;
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
    ParameterPtr(new TintinVCOFrequency(*this, *_programmer, VCO_FREQ1, MOD_M1, MOD_M_FRAC1, MOD_N1)),
    ParameterPtr(new TintinVCOFrequency(*this, *_programmer, VCO_FREQ2, MOD_M2, MOD_M_FRAC2, MOD_N2)),
  }))
    return false;
  
  if(!ToFCamera::_init())
  {
    return false;
  }
  
  if(
    !set(TG_DISABLE, true) ||
    !set(BLK_SIZE, 1024U) ||
    !set(BLK_HEADER_EN, true) ||
    !set(OP_CS_POL, true) ||
    !set(FB_READY_EN, true) ||
    !set(CONFIDENCE_THRESHOLD, 1U) ||
    //!set(DEBUG_EN, true) || // Uncomment this for sample data
    !set(TG_DISABLE, false) ||
    !set(MOD_PLL_UPDATE, true) ||
    !set(MOD_PLL_UPDATE, false))
      return false;
  return true;
}

bool ToFTintinCamera::_initStartParams()
{
  if (!set(TG_DISABLE, false))
    return false;
  return ToFCamera::_initStartParams();
}

bool ToFTintinCamera::_allowedROI(String &message)
{
  message  = "Column start and end must be multiples of 16, both between 0 to 319. ";
  message += "Row start and end must be between 0 to 239.";
  return true;
}

bool ToFTintinCamera::_getROI(RegionOfInterest &roi) const
{
  uint rowStart, rowEnd, colStart, colEnd;
  
  if(!_get(ROW_START, rowStart) || !_get(ROW_END, rowEnd) || !_get(COL_START, colStart) || !_get(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  colStart = colStart*16;
  colEnd = (colEnd + 1)*16 - 1;
  
  roi.x = colStart;
  roi.y = rowStart;
  roi.width = colEnd - colStart + 1;
  roi.height = rowEnd - rowStart + 1;
  return true;
}

bool ToFTintinCamera::_setROI(const RegionOfInterest &roi)
{
  if(isRunning())
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Cannot set frame size while the camera is streaming" << std::endl;
    return false;
  }
  
  uint rowStart, rowEnd, colStart, colEnd;
  
  colStart = (roi.x/16);
  colEnd = ((roi.x + roi.width)/16) - 1;
  
  rowStart = roi.y;
  rowEnd = rowStart + roi.height - 1;
  
  if(!_set(ROW_START, rowStart) || !_set(ROW_END, rowEnd) || !_set(COL_START, colStart) || !_set(COL_END, colEnd))
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Could not get necessary parameters for ROI." << std::endl;
    return false;
  }
  
  if(!_setBinning(1, 1, roi))
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Could not reset binning while setting ROI." << std::endl;
    return false;
  }
  
  FrameSize s;
  if(!_getFrameSize(s) || !_setFrameSize(s, false)) // Get and set frame size to closest possible
  {
    logger(LOG_ERROR) << "ToFTintinCamera: Could not update frame size to closest valid frame size" << std::endl;
    return false;
  }
  
  return true;
}

bool ToFTintinCamera::_isHistogramEnabled() const
{
  return false;
}

bool ToFTintinCamera::_getIlluminationFrequency(float& frequency) const
{
  bool dealiasingEnabled;
  
  uint ma;
  
  if (!get(DEALIAS_EN, dealiasingEnabled))
    return false;
  
  if (!get(MOD_FREQ1, frequency))
    return false;

  if (dealiasingEnabled) {
    if (!get(MA, ma))
      return false;
    frequency /= ma;
  }
  return true;
}

bool ToFTintinCamera::_applyCalibrationParams()
{
  bool commonPhaseCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_COMMON_PHASE_OFFSET_ID)) == 0;
  bool temperatureCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_TEMPERATURE_ID)) == 0;
  bool crossTalkCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_CROSS_TALK_ID)) == 0;
  bool nonlinearityCalibEnable = (configFile.getInteger("calib", CALIB_DISABLE) & (1 << ToF_CALIB_SECT_NON_LINEARITY_ID)) == 0;
  
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
    if(!set(DISABLE_TEMP_CORR, configFile.getBoolean("calib", DISABLE_TEMP_CORR)) ||
      !set(CALIB_PREC, configFile.getBoolean("calib", CALIB_PREC)) ||
      !set(COEFF_ILLUM, configFile.getInteger("calib", COEFF_ILLUM)) ||
      !set(COEFF_SENSOR, configFile.getInteger("calib", COEFF_SENSOR)))
      return false;
  }
  else
  {
    if(!set(DISABLE_TEMP_CORR, true))
      return false;
  }
  
  if(crossTalkCalibEnable && configFile.isPresent("calib", X_CROSS_TALK_COEFF_F1) && configFile.isPresent("calib", Y_CROSS_TALK_COEFF_F1) &&
    configFile.isPresent("calib", X_CROSS_TALK_COEFF_F2) && configFile.isPresent("calib", Y_CROSS_TALK_COEFF_F2))
  {
  
    String crossTalk = 
      configFile.get("calib", X_CROSS_TALK_COEFF_F1) + " " +
      configFile.get("calib", Y_CROSS_TALK_COEFF_F1) + " " +
      configFile.get("calib", X_CROSS_TALK_COEFF_F2) + " " +
      configFile.get("calib", Y_CROSS_TALK_COEFF_F2);
      
    Vector<Complex> coefficients;
    
    coefficients.reserve(4);
    
    std::istringstream ss(crossTalk);
    
    std::copy(std::istream_iterator<Complex>(ss),
              std::istream_iterator<Complex>(),
              std::back_inserter(coefficients));
    
    if(coefficients.size() != 4)
      return false;
    
    float min = 128, max = 0;
    
    for(auto i = 0; i < coefficients.size(); i++)
    {
      if(std::fabs(coefficients[i].real()) < min)
        min = std::fabs(coefficients[i].real());
      
      if(std::fabs(coefficients[i].imag()) < min)
        min = std::fabs(coefficients[i].imag());
      
      if(std::fabs(coefficients[i].real()) > max)
        max = std::fabs(coefficients[i].real());
      
      if(std::fabs(coefficients[i].imag()) > max)
        max = std::fabs(coefficients[i].imag());
    }
    
    uint scale = 3;
    
    min *= 128*4;
    max *= 128*4;
    
    while(max*(1 << scale) > 127 && scale > 0)
      scale--;
    
    uint scale2 = 128*4*(1 << scale);
    
    if(
      !set(CROSS_TALK_FILT_COEFF_X_RE_F1, (int)(coefficients[0].real()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_X_IM_F1, (int)(coefficients[0].imag()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_Y_RE_F1, (int)(coefficients[1].real()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_Y_IM_F1, (int)(coefficients[1].imag()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_X_RE_F2, (int)(coefficients[2].real()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_X_IM_F2, (int)(coefficients[2].imag()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_Y_RE_F2, (int)(coefficients[3].real()*scale2)) ||
      !set(CROSS_TALK_FILT_COEFF_Y_IM_F2, (int)(coefficients[3].imag()*scale2)) ||
      !set(CROSS_TALK_SCALE, scale) ||
      !set(CROSS_TALK_EN, true))
      return false;
  }
  else
  {
    if(!set(CROSS_TALK_EN, false))
      return false;
  }
  
  if(nonlinearityCalibEnable && configFile.isPresent("calib", NONLINEARITY_COEFF_F1) && configFile.isPresent("calib", NONLINEARITY_COEFF_F2) &&
    configFile.isPresent("calib", NONLINEARITY_PHASE_PERIOD))
  {
    Vector<uint> nlCoeff1, nlCoeff2;
    
    nlCoeff1.reserve(16);
    nlCoeff2.reserve(16);
    
    {
      std::istringstream ss(configFile.get("calib", NONLINEARITY_COEFF_F1));
      
      std::copy(std::istream_iterator<uint>(ss),
                std::istream_iterator<uint>(),
                std::back_inserter(nlCoeff1));
    }
    
    {
      std::istringstream ss(configFile.get("calib", NONLINEARITY_COEFF_F2));
      
      std::copy(std::istream_iterator<uint>(ss),
                std::istream_iterator<uint>(),
                std::back_inserter(nlCoeff2));
    }
    
    if(nlCoeff1.size() != 16 || (nlCoeff2.size() != 16 && nlCoeff2.size() != 0))
      return false;
    
    int phasePeriod = configFile.getInteger("calib", NONLINEARITY_PHASE_PERIOD);
    
    String n1 = NONLINEARITY_COEFF_F1, n2 = NONLINEARITY_COEFF_F2;
    
    for(auto i = 0; i < 16; i++)
      if(!set(n1 + "_" + std::to_string(i), nlCoeff1[i]))
        return false;
      
    if(nlCoeff2.size() == 16)
      for(auto i = 0; i < 16; i++)
        if(!set(n2 + "_" + std::to_string(i), nlCoeff2[i]))
          return false;
      
    if(!set(NONLINEARITY_PHASE_PERIOD, phasePeriod) || !set(NONLINEARITY_ENABLE, true))
      return false;
  }
  else
  {
    if(!set(NONLINEARITY_ENABLE, false))
      return false;
  }
  
  return true;
}

bool ToFTintinCamera::_getCurrentProfileRegisterName(String &name)
{
  name = SCRATCH2;
  return true;
}

}
}
