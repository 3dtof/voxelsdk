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

class CalculusIntegDutyCycleSetFailed: public BoolParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
  bool _value = false;
public:
  CalculusIntegDutyCycleSetFailed(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  BoolParameter(programmer, INTG_DUTY_CYCLE_SET_FAILED, 0x5850, 24, 22, {"", ""}, {"", ""}, 0, "", "Indicates that the integration duty cycle is too high.", Parameter::IO_READ_ONLY, {}), _tofCalculusCamera(depthCamera) {}

  virtual bool get(bool &value, bool refresh=false)
  {
    value = _value;
    return true;
  }

  virtual bool set(const bool &value)
  {
    _value = value;
    return true;
  }

  virtual ~CalculusIntegDutyCycleSetFailed() {}
};

class CalculusIntegDutyCycle: public UnsignedIntegerParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
  uint _value = 13;
public:
  CalculusIntegDutyCycle(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, INTEGRATION_DUTY_CYCLE, "", 0x585C, 6, 23, 18, 0, 63, 13, "", "The integration duty cycle can be set using this parameter.", Parameter::IO_READ_WRITE, {}), _tofCalculusCamera(depthCamera) {}

  virtual bool get(uint &value, bool refresh = false)
  {
    value = _value;
    return true;
  }

  virtual bool set(const uint &value)
  {
    _value = value;
    _tofCalculusCamera._updateEasyConf();
    return true;
  }

  virtual ~CalculusIntegDutyCycle() {}
};

class CalculusPixCntMax: public UnsignedIntegerParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
  uint _value = 12500;
public:
  CalculusPixCntMax(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, TM_PIX_CNT_MAX, "", 0x5882, 22, 21, 0, 0, 4194303U, 12500U, "", "The maximum count for master counter. The total quad time period is tm_pix_cnt_max*clk_period. This is made such a way that frame rate under default condition at 24mhz clk is 30 fp/s", Parameter::IO_READ_WRITE, {}), _tofCalculusCamera(depthCamera) {}

  virtual bool get(uint &value, bool refresh = false)
  {
    value = _value;;
    return true;
  }

  virtual bool set(const uint &value)
  {
    _value = value;
    _tofCalculusCamera._updateEasyConf();
    return true;
  }

  virtual ~CalculusPixCntMax() {}
};

class CalculusSubFrameCntMax: public UnsignedIntegerParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
  uint _value = 16;
public:
  CalculusSubFrameCntMax(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, TM_UFRAME_CNT_MAX, "", 0x5883, 7, 10, 4, 0, 127, 16, "", "Total number of sub frames.", Parameter::IO_READ_WRITE, {}), _tofCalculusCamera(depthCamera) {}

  virtual bool get(uint &value, bool refresh = false)
  {
    value = _value;;
    return true;
  }

  virtual bool set(const uint &value)
  {
    _value = value;
    _tofCalculusCamera._updateEasyConf();
    return true;
  }

  virtual ~CalculusSubFrameCntMax() {}
};

class CalculusVCOFrequency: public FloatParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
public:
  CalculusVCOFrequency(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, VCO_FREQ, "MHz", 0, 0, 0, 1, 600, 1300, 864, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, {MOD_M, MOD_N}), _tofCalculusCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    int modN;
    uint modM, systemClockFrequency;
#if 0
    if(!_tofCalculusCamera._get(MOD_M, modM, refresh) || !_tofCalculusCamera._get(MOD_N, modN, refresh))
      return false;
    
    if(modN == 0)
      return false;
    
    float v = systemClockFrequency*modM/modN;
    
    if(!validate(v))
      return false;
    
    value = v;
#endif
    value = 1300;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
#if 0
    if(!_tofCalculusCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _tofCalculusCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    int modN;
    uint modM, systemClockFrequency;
    if(!_tofCalculusCamera._getSystemClockFrequency(systemClockFrequency))
      return false;
    
    modN = 18;
    
    if(systemClockFrequency == 0)
      return false;
    
    modM = value*modN/systemClockFrequency;
    
    if(!_tofCalculusCamera._set(MOD_M, modM) || !_tofCalculusCamera._set(MOD_N, modN))
      return false;
    
    _value = modM*systemClockFrequency/modN;
#endif
    
    return true;
  }
  
  virtual ~CalculusVCOFrequency() {}
};

class CalculusModulationFrequencyParameter: public FloatParameter
{
  ToFCalculusCamera &_tofCalculusCamera;
  String _psName;
public:
  CalculusModulationFrequencyParameter(ToFCalculusCamera &depthCamera, RegisterProgrammer &programmer, const String &name, const String &psName):
  FloatParameter(programmer, name, "MHz", 0, 0, 0, 1, 6.25f, 433.333f, 18, "Modulation frequency", "Frequency used for modulation of illumination", 
                 Parameter::IO_READ_WRITE, {psName}), _psName(psName), _tofCalculusCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = false)
  {
    float vcoFrequency;
    
    uint modulationPS;
    
#if 0
    if(!_tofCalculusCamera._get(VCO_FREQ, vcoFrequency, refresh) || !_tofCalculusCamera._get(_psName, modulationPS, refresh))
      return false;
    
    float v = vcoFrequency/3/modulationPS;
    
    if(!validate(v))
      return false;
    
    value = v;
#endif
    value = 19.2f;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
#if 0
    ParameterPtr p = _tofCalculusCamera.getParam(VCO_FREQ);
    
    if(!p)
      return false;
    
    CalculusVCOFrequency &v = (CalculusVCOFrequency &)*p;
    
    uint modulationPS = v.upperLimit()/3/value;
    
    if(!v.set(modulationPS*3*value))
      return false;
    
    if(!_tofCalculusCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _tofCalculusCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    if(!_tofCalculusCamera._set(_psName, modulationPS))
      return false;
    
    float val;
    
    if(!v.get(val))
      return false;
#endif
    
    return true;
  }
};

ToFCalculusCamera::ToFCalculusCamera(const String &name, DevicePtr device): ToFCamera(name, device)
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

  
#define EN_EASY_CONF_REG                0x585B
#define TM_PIX_CNT_MAX_REG              0x5882
#define TM_UFRAME_CNT_MAX_REG           0x5883
#define TM_RDOUT_PHASE_PIX2_REG         0X58AA
#define TM_INTG_PHASE_PIX2_REG          0x58AF
#define TM_SPARE3_PIX1_REG              0x58CC
#define TM_SPARE3_PIX2_REG              0x58CD
#define TM_SPARE3_PAT1_REG              0x58CE
#define TM_SPARE3_INIT_REG              0x58D0
#define TM_RDOUT_VD_PIX1_REG            0x58D1
#define TM_RDOUT_VD_PIX2_REG            0x58D2
#define TM_RDOUT_HD_CNT_START_PIX1_REG  0x588E
#define TM_RDOUT_HD_CNT_END_PIX1_REG    0x588F
#define CLIP_MODE_REG                   0x58F6

bool ToFCalculusCamera::_updateEasyConf()
{
  uint intg_duty_cycle, pix_cnt_max, sub_frame_cnt_max;
  
  // This represents the original value of intg_phase_pix2, and we pull in the register value by 50 cycles
  // other values are pushed out relative to this original value
  uint tm_intg_phase_pix2_int, tm_intg_phase_pix2_limit;
  bool tg_en_value;
  uint total_length_of_rdout_frame = 4950U;
  bool pix_cnt_overflow = false, intg_phase_overflow = false;

  _get(INTEGRATION_DUTY_CYCLE, intg_duty_cycle);
  _get(TM_PIX_CNT_MAX, pix_cnt_max);
  _get(TM_UFRAME_CNT_MAX, sub_frame_cnt_max);
  _get(TG_EN, tg_en_value);

  tm_intg_phase_pix2_int = 722 + ((pix_cnt_max * intg_duty_cycle) >> 6);

  if (pix_cnt_max < (total_length_of_rdout_frame + 922U))
    pix_cnt_overflow = true;

  if (pix_cnt_overflow)
    tm_intg_phase_pix2_limit = 723U;
  else
    tm_intg_phase_pix2_limit = pix_cnt_max - (total_length_of_rdout_frame + 300);

  if (tm_intg_phase_pix2_limit < tm_intg_phase_pix2_int)
    intg_phase_overflow = true;
  else
    intg_phase_overflow = false;

  if (intg_phase_overflow) {
    tm_intg_phase_pix2_int = tm_intg_phase_pix2_limit;
    _set(INTG_DUTY_CYCLE_SET_FAILED, true);
    return true;
  }
  
  _set(INTG_DUTY_CYCLE_SET_FAILED, false);
  _set(TG_EN, false);
  // Set INTG_PHASE_PIX2 and RDOUT_PHASE_PIX2 to 50 cycles earlier than usual, and gate shutter at the usual point
  _programmer->writeRegister(TM_INTG_PHASE_PIX2_REG, tm_intg_phase_pix2_int - 50);
  _programmer->writeRegister(TM_RDOUT_PHASE_PIX2_REG, tm_intg_phase_pix2_int - 50);
  _programmer->writeRegister(TM_SPARE3_PIX2_REG, tm_intg_phase_pix2_int);
  _programmer->writeRegister(TM_RDOUT_HD_CNT_START_PIX1_REG, tm_intg_phase_pix2_int + 100);
  _programmer->writeRegister(TM_RDOUT_VD_PIX1_REG, 0x400000 + tm_intg_phase_pix2_int + 100);
  _programmer->writeRegister(TM_RDOUT_VD_PIX2_REG, tm_intg_phase_pix2_int + 100 + 8);
  _programmer->writeRegister(TM_RDOUT_HD_CNT_END_PIX1_REG, tm_intg_phase_pix2_int + 100 + 4950);

  _programmer->writeRegister(TM_PIX_CNT_MAX_REG, pix_cnt_max);
  _programmer->writeRegister(TM_UFRAME_CNT_MAX_REG, 4 + (sub_frame_cnt_max << 4));
  _set(TG_EN, tg_en_value);

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
    ParameterPtr(new CalculusModulationFrequencyParameter(*this, *_programmer, MOD_FREQ1, MOD_PS1)),
    ParameterPtr(new CalculusIntegDutyCycle(*this, *_programmer)),
    ParameterPtr(new CalculusPixCntMax(*this, *_programmer)),
    ParameterPtr(new CalculusSubFrameCntMax(*this, *_programmer)),
    ParameterPtr(new CalculusIntegDutyCycleSetFailed(*this, *_programmer)),
  }))
    return false;
  
  if(!ToFCamera::_init())
  {
    return false;
  }
  
  return true;
}


bool ToFCalculusCamera::_initStartParams()
{
#if 0
  if(!set(TG_EN, true) || 
     !set(BLK_SIZE, 1024U) ||
     !set(BLK_HEADER_EN, true) ||
     !set(OP_CS_POL, true) ||
     !set(FB_READY_EN, true) ||
     !set(CONFIDENCE_THRESHOLD, 1U) ||
     //!set(DEBUG_EN, true) || // Uncomment this for sample data
     !set(ILLUM_EN_POL, false) ||
     !getROI(roi) ||
     !getFrameSize(s))
    // || set(INTG_TIME, 40.0f);
#endif

#if 0
  if (!set(EN_GLOBAL_PDN_REG, true) ||
      !set(EN_GLOBAL_PDN_REG, false) ||
      !set(DIS_SDMOD, true) ||
      !set(ADCK_R, 10U) ||
      !set(UPDATE_SEL, 2U) ||
      !set(SYNC_INTEG_PHASE_EN, true) ||
      !set(TM_PIX_CNT_MAX, 50000U) ||
      !set(TM_UFRAME_CNT_MAX, 4U) ||
      !set(INTEGRATION_DUTY_CYCLE, 35U) ||
      !set(TG_EN, true) ||
      !set(TM_INTG_PHASE_PIX1, 284U) ||
      !set(DVP_BLANK_PIX_PER_LINE, 1023U) ||
      !set(DISABLE_REARRANGE, true) ||
      !set(MOD_PS1, 4) ||
      !set(MOD_PLL_UPDATE, true) ||
      !set(MOD_PLL_UPDATE, false) ||
      !set(TM_PDZ_LED_DRIVER, true) ||
      !set(LED_DRIVER_CURR_PROG, 30U) ||
      !set(MCKADGEN_R_OFFSET, 10U) ||
      !set(SHUTTER_EN, true) ||
      !set(TEST_PG1, true) ||
      !set(FLIP_PHASE, true) ||
      !set(CALIB_PHASE_OFFSET, -2884) || 
      !getROI(roi) ||
      !getFrameSize(s)
    )
    return false;
#else
  if (!_programmer->writeRegister(0x5808, 0x000004) || //!set(EN_GLOBAL_PDN_REG, true) ||
      !_programmer->writeRegister(0x5808, 0x000000) || //!set(EN_GLOBAL_PDN_REG, false) ||
      !_programmer->writeRegister(0x580E, 0x000001) || //!set(DIS_SDMOD, true) ||
      !_programmer->writeRegister(0x5812, 0x00000A) || //!set(ADCK_R, 10U) ||
      !_programmer->writeRegister(0x5881, 0x0000A0) || //!set(UPDATE_SEL, 2U) ||
      !_programmer->writeRegister(0x580b, 0x002000) || //!set(SYNC_INTEG_PHASE_EN, true) ||
      !_programmer->writeRegister(EN_EASY_CONF_REG, 0x000001) || // Disable Easy Conf
      !_programmer->writeRegister(TM_SPARE3_PIX1_REG, 0x400003) || // SPARE3_POL1 = 1
      !_programmer->writeRegister(TM_SPARE3_PAT1_REG, 0x400000) || // SPARE3_PAT1 = 01
      !_programmer->writeRegister(TM_SPARE3_INIT_REG, 0x000081) || // SPARE3_INIT = 0
      //!_programmer->writeRegister(0x58AE, 0x40011C) || //!set(TM_INTG_PHASE_PIX1, 284U) ||
      //!_programmer->writeRegister(0x58DC, 0x5FFA80) || //!set(DVP_BLANK_PIX_PER_LINE, 1023U) ||
      !_programmer->writeRegister(0x585C, 0x8C1000) || //!set(DISABLE_REARRANGE, true) ||
      !set(MOD_PS1, 4U) ||
      !set(MOD_PLL_UPDATE, true))
      return false;
      
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (
      !set(MOD_PLL_UPDATE, false) ||
      !set(TM_PDZ_LED_DRIVER, true) ||
      !set(LED_DRIVER_CURR_PROG, 30U) ||
      !_programmer->writeRegister(0x581B, 0x00000A) || //!set(MCKADGEN_R_OFFSET, 10U) ||
      //!_programmer->writeRegister(0x585B, 0xC00001) || //!set(SHUTTER_EN, true) ||
      !_programmer->writeRegister(0x58D6, 0x400001) || //!set(TEST_PG1, true) ||
      !_programmer->writeRegister(0x58F9, 0x080000) || //!set(FLIP_PHASE, true) ||
      !set(CALIB_PHASE_OFFSET, -2900) || 
      !_programmer->writeRegister(CLIP_MODE_REG, 0x880000) // !set(CLIP_MODE, true) ||  // Check if we can move this out of the DML and hardcode instead
    )
    return false;

  if (
      !set(TM_PIX_CNT_MAX, 50000U) ||
      !set(TM_UFRAME_CNT_MAX, 4U) ||
      !set(INTEGRATION_DUTY_CYCLE, 55U) ||
      !set(TG_EN, true)
    )
    return false;

#endif
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
  //bool histogramEnabled;
  return false;
  //return _get(HISTOGRAM_EN, histogramEnabled) && histogramEnabled;
}

bool ToFCalculusCamera::_getToFFrameType(ToFFrameType &frameType) const
{
  frameType = ToF_PHASE_AMPLITUDE;
  return true;
}

bool ToFCalculusCamera::_getIlluminationFrequency(float& frequency) const
{
  float modulationFrequency1, modulationFrequency2;
  bool dealiasingEnabled;
  
  uint modulationPS1, modulationPS2;
  
#if 0
  if(!get(DEALIAS_EN, dealiasingEnabled))
    return false;
  
  if(dealiasingEnabled)
  {
    if(!get(MOD_FREQ1, modulationFrequency1) || !get(MOD_FREQ2, modulationFrequency2)) // ensure that these are valid
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
#endif
  frequency = 19.2;
  return true;
}

bool ToFCalculusCamera::_is16BitModeEnabled(bool &mode16Bit)
{
  mode16Bit = false;
  return true;
}

bool ToFCalculusCamera::_applyCalibrationParams()
{
  return true;
}

 
}
}