/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFHaddockCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

namespace Voxel
{
  
namespace TI
{
  
/// Custom parameters
class VCOFrequency: public FloatParameter
{
  ToFHaddockCamera &_depthCamera;
public:
  VCOFrequency(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, VCO_FREQ, "MHz", 0, 0, 0, 1, 600, 1300, 864, "VCO frequency", 
                 "Frequency of the VCO used for generating modulation frequencies", IOType::IO_READ_WRITE, {MOD_M, MOD_N}), _depthCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = true) const
  {
    uint modM, modN, systemClockFrequency;
    if(!_depthCamera._get(MOD_M, modM) || !_depthCamera._get(MOD_N, modN) || !_depthCamera._get(SYS_CLK_FREQ, systemClockFrequency))
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
    
    if(!_depthCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    uint modM, modN, systemClockFrequency;
    if(!_depthCamera._get(SYS_CLK_FREQ, systemClockFrequency))
      return false;
    
    modN = 18;
    
    if(systemClockFrequency == 0)
      return false;
    
    modM = value*modN/systemClockFrequency;
    
    if(!_depthCamera._set(MOD_M, modM) || !_depthCamera._set(MOD_N, modN))
      return false;
    
    _value = modM*systemClockFrequency/modN;
    
    return true;
  }
  
  virtual ~VCOFrequency() {}
};

class ModulationFrequencyParameter: public FloatParameter
{
  ToFHaddockCamera &_depthCamera;
  String _psName;
public:
  ModulationFrequencyParameter(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer, const String &name, const String &psName):
  FloatParameter(programmer, name, "MHz", 0, 0, 0, 1, 6.25f, 433.333f, 18, "Modulation frequency", "Frequency used for modulation of illumination", 
                 Parameter::IO_READ_WRITE, {psName}), _psName(psName), _depthCamera(depthCamera) {}
                 
  virtual bool get(float &value, bool refresh = true) const
  {
    float vcoFrequency;
    
    uint modulationPS;
    
    if(!_depthCamera._get(VCO_FREQ, vcoFrequency) || !_depthCamera._get(_psName, modulationPS))
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
    
    ParameterPtr p = _depthCamera.getParam(VCO_FREQ);
    
    if(!p)
      return false;
    
    VCOFrequency &v = (VCOFrequency &)*p;
    
    uint modulationPS = v.upperLimit()/3/value;
    
    if(!v.set(modulationPS*3*value))
      return false;
    
    if(!_depthCamera._set(MOD_PLL_UPDATE, true))
      return false;
    
    ParameterPtr pllUpdate(nullptr, [this](Parameter *) { _depthCamera._set(MOD_PLL_UPDATE, false); }); // Set PLL update to false when going out of scope of this function
    
    if(!_depthCamera._set(_psName, modulationPS))
      return false;
    
    float val;
    
    if(!v.get(val))
      return false;
    
    return true;
  }
};

class IntegrationTimeParameter: public FloatParameter
{
  ToFHaddockCamera &_depthCamera;
public:
  IntegrationTimeParameter(ToFHaddockCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, INTG_TIME, "%", 0, 0, 0, 1, 0, 100, 0, "Integration time", 
                "Integration time as percentage of total cycle time", Parameter::IO_READ_WRITE, {INTG_DUTY_CYCLE}), _depthCamera(depthCamera) {}
                
  virtual bool get(float &value, bool refresh = true) const
  {
    uint integrationDutyCycle;
    
    bool integrationDutyCycleSetFailed;
    
    if(!_depthCamera._get(INTG_DUTY_CYCLE, integrationDutyCycle) || 
      !_depthCamera._get(INTG_DUTY_CYCLE_SET_FAILED, integrationDutyCycleSetFailed)
      || integrationDutyCycleSetFailed)
      return false;
    
    
    float v = integrationDutyCycle*100/63;
    
    if(v > 100) v = 100;
    if(v < 0) v = 0;
    
    value = v;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    uint integrationDutyCycle = (value/100)*63;
    
    if(integrationDutyCycle > 63) integrationDutyCycle = 63;
    
    if(!_depthCamera._set(INTG_DUTY_CYCLE, integrationDutyCycle))
      return false;
    
    bool integrationDutyCycleSetFailed;
    
    if(!_depthCamera._get(INTG_DUTY_CYCLE_SET_FAILED, integrationDutyCycleSetFailed)
      || integrationDutyCycleSetFailed)
      return false;
    
    return true;
  }
};
  

ToFHaddockCamera::ToFHaddockCamera(const String &name, DevicePtr device): ToFCamera(name, device)
{
}


  
bool ToFHaddockCamera::_init()
{
  Configuration c;
  
  String name = "OPT9220A.dml"; // TODO: This needs to come from a configuration
  
  if(!c.getConfFile(name)) // true => name is now a proper path
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Failed to locate/read DML file '" << name << "'" << std::endl;
    return false;
  }
  
  ParameterDMLParser p(*_programmer, name);
  
  Vector<ParameterPtr> params;
  
  if(!p.getParameters(params))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not read parameters from DML file '" << name << "'" << std::endl;
    return false;
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
    ParameterPtr(new VCOFrequency(*this, *_programmer)),
    ParameterPtr(new ModulationFrequencyParameter(*this, *_programmer, MOD_FREQ1, MOD_PS1)),
    ParameterPtr(new ModulationFrequencyParameter(*this, *_programmer, MOD_FREQ2, MOD_PS2)),
    ParameterPtr(new IntegrationTimeParameter(*this, *_programmer))
  }))
    return false;
  
  
  return true;
}

bool ToFHaddockCamera::_getFrameRate(FrameRate &r) const
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, pixCount, sysClkFrequency;
  
  if(!_get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  if(!_get(PIX_CNT_MAX, pixCount) || !_get(QUAD_CNT_MAX, quadCount) || !_get(SUBFRAME_CNT_MAX, subFrameCount) || !_get(SYS_CLK_FREQ, sysClkFrequency))
    return false;
  
  uint numerator = sysClkFrequency*1000000,
  denominator = pixCount*quadCount*subFrameCount;
  
  uint g = gcd(numerator, denominator);
  
  r.numerator = numerator/g;
  r.denominator = denominator/g;
  return true;
}

bool ToFHaddockCamera::_setFrameRate(const FrameRate &r)
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, sysClkFrequency, pixCount;
  
  if(!_get(QUAD_CNT_MAX, quadCount) || !_get(SUBFRAME_CNT_MAX, subFrameCount) || !_get(SYS_CLK_FREQ, sysClkFrequency))
    return false;
  
  pixCount = (uint)(((long)r.denominator*sysClkFrequency*1000000)/((long)quadCount*subFrameCount*r.numerator));
  
  logger(LOG_DEBUG) << "ToFHaddockCamera: Setting " << PIX_CNT_MAX << " = " << pixCount << std::endl;
  
  if(!_set(PIX_CNT_MAX, pixCount) || !_get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  return true;
}

bool ToFHaddockCamera::_getFrameSize(Voxel::FrameSize &s) const
{
  uint binRowCount, binColumnCount;
  
  if(!_get(BIN_ROW_COUNT, binRowCount) || !_get(BIN_COLUMN_COUNT, binColumnCount))
    return false;
  
  s.width = binColumnCount;
  s.height = binRowCount;
  return true;
}


bool ToFHaddockCamera::_setFrameSize(const FrameSize &s)
{
  return true; // dummy to be coded later
}


bool ToFHaddockCamera::_initStartParams()
{
  return set(TG_EN, true) && 
         set(BLK_SIZE, 1024U) &&
         set(BLK_HEADER_EN, true) &&
         set(OP_CS_POL, true) &&
         set(FB_READY_EN, true) &&
         set(CONFIDENCE_THRESHOLD, 1U) &&
         //set(DEBUG_EN, true) && // Uncomment this for sample data
         set(ILLUM_EN_POL, false);
         // && set(INTG_TIME, 40.0f);
}

// FIXME: Should amplitude_scale parameter be used for this?
bool ToFHaddockCamera::_getAmplitudeNormalizingFactor(float &factor)
{
  factor = 1.0/(1 << 12);
  return true;
}

// FIXME: Should frequency_scale parameter be used for this?
bool ToFHaddockCamera::_getDepthScalingFactor(float &factor)
{
  float modulationFrequency1, modulationFrequency2;
  bool dealiasingEnabled;
  
  uint modulationPS1, modulationPS2;
  
  if(!get(DEALIAS_EN, dealiasingEnabled))
    return false;
  
  if(dealiasingEnabled)
  {
    if(!get(MOD_FREQ1, modulationFrequency1) || !get(MOD_FREQ2, modulationFrequency2)) // ensure that these are valid
      return false;
    
    if(!get(MOD_PS1, modulationPS1) || !get(MOD_PS2, modulationPS2))
      return false;
    
    float freq = modulationFrequency1*gcd(modulationPS1, modulationPS2)/modulationPS2;
    
    factor = SPEED_OF_LIGHT/1E6f/(2*(1 << 12)*freq);
    
    return true;
  }
  else
  {
    if(!get(MOD_FREQ1, modulationFrequency1))
      return false;
    
    factor = SPEED_OF_LIGHT/1E6f/2/modulationFrequency1/(1 << 12);
    return true;
  }
}


bool ToFHaddockCamera::_processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput)
{
  FrameSize s;
  
  RawDataFramePtr rawDataFrame = std::dynamic_pointer_cast<RawDataFrame>(rawFrameInput);
  
  if(!rawDataFrame)
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Input data frame is not of raw data type." << std::endl;
    return false;
  }
  
  if(!getFrameSize(s))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Could not get the current frame size. Cannot convert raw data to ToF data" << std::endl;
    return false;
  }
  
  int bytesPerPixel, dataArrangeMode;
  if(!get(PIXEL_DATA_SIZE, bytesPerPixel) || !get(OP_DATA_ARRANGE_MODE, dataArrangeMode))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Failed to read " << PIXEL_DATA_SIZE << " or " << OP_DATA_ARRANGE_MODE << std::endl;
    return false;
  }
  
  ToFRawFrameTemplate<uint16_t, uint8_t> *t;
  
  if(!rawFrameOutput)
  {
    t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
    rawFrameOutput = RawFramePtr(t);
  }
  else
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(rawFrameOutput.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      rawFrameOutput = RawFramePtr(t);
    }
  }
  
  t->size = s;
  t->id = rawDataFrame->id;
  t->timestamp = rawDataFrame->timestamp;
  
  if(bytesPerPixel == 4)
  {
    if(rawDataFrame->data.size() < s.height*s.width*4)
    {
      logger(LOG_ERROR) << "ToFHaddockCamera: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << s.height*s.width*4 << std::endl;
      return false;
    }
    
    if(dataArrangeMode != 2 && dataArrangeMode != 0)
    {
      logger(LOG_ERROR) << "ToFHaddockCamera: Invalid op_data_arrange_mode = " << dataArrangeMode << " for pixel_data_size = " << bytesPerPixel << std::endl;
      return false;
    }
    
    t->_ambient.resize(s.width*s.height);
    t->_amplitude.resize(s.width*s.height);
    t->_phase.resize(s.width*s.height);
    t->_flags.resize(s.width*s.height);
    
    if(dataArrangeMode == 2)
    {
      auto index1 = 0, index2 = 0;
      
      uint16_t *data = (uint16_t *)rawDataFrame->data.data();
      
      for (auto i = 0; i < s.height; i++) 
      {
        for (auto j = 0; j < s.width/8; j++) 
        {
          index1 = i*s.width*2 + j*16;
          index2 = i*s.width + j*8;
          
          //logger(INFO) << "i = " << i << ", j = " << j << ", index1 = " << index1 << ", index2 = " << index2 << std::endl;
          
          for (auto k = 0; k < 8; k++) 
          {
            t->_amplitude[index2 + k] = data[index1 + k] & 0x0FFF;
            t->_ambient[index2 + k] = (data[index1 + k] & 0xF000) >> 12;
            
            t->_phase[index2 + k] = data[index1 + k + 8] & 0x0FFF;
            t->_flags[index2 + k] = (data[index1 + k + 8] & 0xF000) >> 12;
          }
        }
      }
    }
    else // dataArrangeMode == 0
    {
      auto index1 = 0, index2 = 0;
      
      uint16_t *data = (uint16_t *)rawDataFrame->data.data();
      
      for (auto i = 0; i < s.height; i++) 
      {
        for (auto j = 0; j < s.width; j++) 
        {
          index1 = i*s.width*2 + j*2;
          index2 = i*s.width + j;
          t->_amplitude[index2] = data[index1] & 0x0FFF;
          t->_ambient[index2] = (data[index1] & 0xF000) >> 12;
          
          t->_phase[index2] = data[index1] & 0x0FFF;
          t->_flags[index2] = (data[index1] & 0xF000) >> 12;
        }
      }
    }
  }
  else if(bytesPerPixel == 2)
  {
    if(dataArrangeMode != 0)
    {
      logger(LOG_ERROR) << "ToFHaddockCamera: " << OP_DATA_ARRANGE_MODE << " is expected to be zero, but got = " << dataArrangeMode << " for " << PIXEL_DATA_SIZE << " = " << bytesPerPixel << std::endl;
      return false;
    }
    
    if(rawDataFrame->data.size() < s.height*s.width*2)
    {
      logger(LOG_ERROR) << "ToFHaddockCamera: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << s.height*s.width*2 << std::endl;
      return false;
    }
    
    t->_ambient.resize(s.width*s.height);
    t->_amplitude.resize(s.width*s.height);
    t->_phase.resize(s.width*s.height);
    t->_flags.resize(s.width*s.height);
    
    auto index = 0;
    
    uint16_t *data = (uint16_t *)rawDataFrame->data.data();
    
    for (auto i = 0; i < s.height; i++) 
    {
      for (auto j = 0; j < s.width; j++) 
      {
        index = i*s.width + j;
        t->_phase[index] = data[index] & 0x0FFF;
        t->_amplitude[index] = (data[index] & 0xF000) >> 4; // Amplitude information is MS 4-bits
        
        t->_ambient[index] = 0;
        t->_flags[index] = 0;
      }
    }
  }
  else
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Don't know to handle " << PIXEL_DATA_SIZE << " = " << bytesPerPixel << std::endl;
    return false;
  }
  
  bool histogramEnabled;
  if(!get(HISTOGRAM_EN, histogramEnabled))
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Failed to find whether histogram is enabled or not" << std::endl;
    return false;
  }
  
  if(!histogramEnabled)
    return true; // No histogram data
  
  if(rawDataFrame->data.size() < s.height*s.width*bytesPerPixel + 96)
  {
    logger(LOG_ERROR) << "ToFHaddockCamera: Histogram is enabled but raw data has less than 96 bytes at the end. Raw data size = " << rawDataFrame->data.size() 
      << ", bytes in frame = " << s.height*s.width*bytesPerPixel << std::endl;
    return false;
  }
  
  uint8_t *data = rawDataFrame->data.data() + s.height*s.width*bytesPerPixel;
  
  t->_histogram.resize(48); // 48 elements of 16-bits each
  
  memcpy((uint8_t *)t->_histogram.data(), data, 96);
  
  return true;
}
 
}
}