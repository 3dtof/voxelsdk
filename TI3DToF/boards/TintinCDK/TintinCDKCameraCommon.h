/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#ifndef TINTINCDKCAMERACOMMON_H
#define TINTINCDKCAMERACOMMON_H

#include <Parameter.h>
#include <ToFTintinCamera.h>
#include <RegisterProgrammer.h>
#include "VoxelXUProgrammer.h"
#include "VoxelUSBProgrammer.h"
#define _USE_MATH_DEFINES
#include<math.h>

#define PVDD "pvdd" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage
#define TILLUM_SLAVE_ADDR "tillum_slv_addr"
#define LUMPED_DEAD_TIME "lumped_dead_time"
#define ILLUM_DC_CORR "illum_dc_corr"
#define ILLUM_DC_CORR_DIR "illum_dc_corr_dir"
#define DELAY_FB_CORR_MODE "delay_fb_corr_mode"
#define DELAY_FB_DC_CORR_MODE "delay_fb_dc_corr_mode"
#define COMP_VREF "comp_vref"

namespace Voxel
{
namespace TI
{



class TintinCDKMixVoltageParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    if(value > 0x80U)
      return (value - 0x80U)*50 + 500;
    else
      return 500;
  }

  virtual uint32_t _toRawValue(uint value) const
  {
    if(value > 500)
      return (value - 500)/50 + 0x80U;
    else
      return 0x80U;
  }

public:
  TintinCDKMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 1200, 2000, 1500, "Mixing voltage",
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~TintinCDKMixVoltageParameter() {}
};

class TintinCDKPVDDParameterRev2: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    if (value > 0xBC)
      return (value - 0xBC)*100 + 3000;
    else if (value > 0xA0)
      return (value - 0xA0)*50 + 1600;
    else if(value > 0x80U)
      return (value - 0x80U)*25 + 800;
    else
      return 800;
  }

  virtual uint32_t _toRawValue(uint value) const
  {
    if(value >= 3000)
      return (value - 3000)/100 + 0xBCU;
    else if(value >= 1600)
      return (value - 1600)/50 + 0xA0U;
    else if(value >= 800)
      return (value - 800)/25 + 0x80U;
    else
      return 0x80U;
  }

public:
  TintinCDKPVDDParameterRev2(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, PVDD, "mV", 0x2D1E, 8, 7, 0, 2000, 3300, 3300, "Pixel VDD",
                           "Reset voltage level of pixel.", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~TintinCDKPVDDParameterRev2() {}
};

class TintinCDKIlluminationPowerParameter: public UnsignedIntegerParameter
{
protected:
  // Relations derived by Subhash
  virtual uint _fromRawValue(uint32_t value) const
  {
    float R = value*100.0/255; //Digipot resistance in kOhms for given register setting
    float Rlim = R*113.0/(R+113)+51.1; //Effective current limiting resistor in kOhms for the given digipot resistance
    float I = 118079.0e-3/Rlim; //Current limit implemented by the linear current limiting IC for the given limiting resistance. This has a small variation as compared to the datasheet. That assumption has been made for the purpose of simplification.

    uint retVal = uint(4*(I-0.2)*1100); //Subtracting the threshold current and multiplying by the power gain of the laser diode.

    if (retVal < lowerLimit())
      return lowerLimit();
    else if (retVal > upperLimit())
      return upperLimit();
    else
      return retVal;
  }

  virtual uint32_t _toRawValue(uint value) const
  {
    float v = value/1000.0/4;
    return uint32_t(((130000/(v + 0.22))-51.1e3)*28815e3/((164.1e3 - 130000/(v + 0.22))*100e3));
  }

public:
  TintinCDKIlluminationPowerParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER, "mW", 0x5401, 8, 7, 0, 4*1000, 4*2200, 4*1129, "Illumination Power",
                           "These power numbers are approximate (+- 20%) and the actual power numbers are subject to component tolerances.", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~TintinCDKIlluminationPowerParameter() {}
};


class TintinCDKIlluminationPowerPercentParameter: public UnsignedIntegerParameter
{
protected:
  ToFTintinCamera &_depthCamera;

public:
  TintinCDKIlluminationPowerPercentParameter(ToFTintinCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER_PERCENTAGE, "%", 0, 0, 0, 0, 48, 100, 51, "Illumination Power",
                           "These power numbers are approximate (+- 20%) and the actual power numbers are subject to component tolerances.", Parameter::IO_READ_WRITE, {}),
                           _depthCamera(depthCamera)
  {}

  virtual bool get(uint &value, bool refresh = false)
  {
    uint v;
    TintinCDKIlluminationPowerParameter *p = dynamic_cast<TintinCDKIlluminationPowerParameter *>(_depthCamera.getParam(ILLUM_POWER).get());
    if(!p || !p->get(v))
      return false;

    value = (uint)((v*100.0f)/p->upperLimit() + 0.5); // Rounded value
    return true;
  }

  virtual bool set(const uint &value)
  {
    TintinCDKIlluminationPowerParameter *p = dynamic_cast<TintinCDKIlluminationPowerParameter *>(_depthCamera.getParam(ILLUM_POWER).get());
    if(!p)
      return false;

    uint v = (value*p->upperLimit())/100;

    if(!p->set(v))
      return false;

    return true;
  }

  virtual ~TintinCDKIlluminationPowerPercentParameter() {}
};

class TintinCDKIllumVrefParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    return (3300U * value / 255U);
  }

  virtual uint32_t _toRawValue(uint value) const
  {
    return (255U * value / 3300U);
  }

public:
  TintinCDKIllumVrefParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, COMP_VREF, "mV", 0x5400, 8, 7, 0, 0, 3300, 1405, "Comp Vref",
                           "This voltage is the reference voltage used for comparing the laser voltage in the illumination delay compensation loop.", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~TintinCDKIllumVrefParameter() {}
};

class TintinCDKIllumCurrentParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    uint8_t lsbyte = value & 0xFF;
    uint8_t msbyte = (value >> 8) & 0xFF;
    uint current = (lsbyte << 8) + msbyte;
    // Calibration register is set to 6991 to get about 122.07 uA/bit
    return (current * 0.12207);
  }

//   virtual uint32_t _toRawValue(uint value) const
//   {
//     return (255U * value / 3300U);
//   }

public:
  TintinCDKIllumCurrentParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, "illum_current", "mA", 0x4E04, 8, 7, 0, 0, 8000, 0000, "Illumination Current",
                           "This is the current on the illumination 5V rail.", Parameter::IO_READ_ONLY, {})
  {}

  virtual ~TintinCDKIllumCurrentParameter() {}
};

class TintinCDKMainCurrentParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    uint8_t lsbyte = value & 0xFF;
    uint8_t msbyte = (value >> 8) & 0xFF;
    uint current = (lsbyte << 8) + msbyte;

    // Calibration register is set to 1864 to get about 61.035 uA/bit
    return (current * 0.061035);
  }

//   virtual uint32_t _toRawValue(uint value) const
//   {
//     return (255U * value / 3300U);
//   }

public:
  TintinCDKMainCurrentParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, "main_current", "mA", 0x4B04, 8, 7, 0, 0, 2000, 0000, "Main Board Current",
                           "This is the current drawn by the main board.", Parameter::IO_READ_ONLY, {})
  {}

  virtual ~TintinCDKMainCurrentParameter() {}
};

class TintinCDKModulationFrequencyParameter: public TintinModulationFrequencyParameter
{
public:
  TintinCDKModulationFrequencyParameter(ToFTintinCamera &depthCamera, RegisterProgrammer &programmer, const String &name, const String &vcoFreq, const String &modPS, const float &defaultValue = 48):
    TintinModulationFrequencyParameter(depthCamera, programmer, name, vcoFreq, modPS, defaultValue) {}

  virtual const float getOptimalMaximum() { return 60; }
  virtual const float getOptimalMinimum() { return 39; }

  virtual ~TintinCDKModulationFrequencyParameter() {}
};

#define DEFAULT_UNAMBIGUOUS_RANGE 4095*SPEED_OF_LIGHT/1E6f/2/48/(1 << 12)
class TintinCDKUnambiguousRangeParameter: public TintinUnambiguousRangeParameter
{
public:
  TintinCDKUnambiguousRangeParameter(ToFTintinCamera& depthCamera, RegisterProgrammer& programmer):
    TintinUnambiguousRangeParameter(depthCamera, programmer, 3, 50, DEFAULT_UNAMBIGUOUS_RANGE, 1, 0) {}
  virtual ~TintinCDKUnambiguousRangeParameter() {}
};
}
}
#endif
