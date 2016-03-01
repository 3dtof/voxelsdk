/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "TintinCDKCamera.h"
#include "VoxelXUProgrammer.h"
#include "VoxelUSBProgrammer.h"
#include <Logger.h>
#include <UVCStreamer.h>
#include <USBBulkStreamer.h>

#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define REQUEST_EEPROM_SIZE 0x31
#define REQUEST_EEPROM_DATA 0x33

namespace Voxel
{
  
namespace TI
{
  
TintinCDKCamera::TintinCDKCamera(Voxel::DevicePtr device): ToFTintinCamera("TintinCDKCamera", device)
{
  _init();
}

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


class TintinCDKPVDDParameterRev1: public UnsignedIntegerParameter
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
  TintinCDKPVDDParameterRev1(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, PVDD, "mV", 0x2D0E, 8, 7, 0, 2000, 3600, 3300, "Pixel VDD", 
                           "Reset voltage level of pixel.", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~TintinCDKPVDDParameterRev1() {}
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
  TintinCDKCamera &_depthCamera;
  
public:
  TintinCDKIlluminationPowerPercentParameter(TintinCDKCamera &depthCamera, RegisterProgrammer &programmer):
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

class TintinCDKDummyDelayFBCorrModeParameter: public IntegerParameter
{
  int _value = 0;
public:
  TintinCDKDummyDelayFBCorrModeParameter(RegisterProgrammer &programmer):
  IntegerParameter(programmer, DELAY_FB_CORR_MODE, "", 0x5CB1, 24, 22, 21, -2, 1, 0, "", "Dummy parameter for delay fb correction (rev1 CDK only).", Parameter::IO_READ_WRITE, {}) {}
  virtual bool get(int &value, bool refresh = false)
  {
    value = _value;
    return true;
  }

  virtual bool set(const int &value)
  {
    _value = value;
    return true;
  }

  virtual ~TintinCDKDummyDelayFBCorrModeParameter() {}
};

class TintinCDKDummyDelayFBDCCorrModeParameter: public IntegerParameter
{
  int _value = 0;
public:
  TintinCDKDummyDelayFBDCCorrModeParameter(RegisterProgrammer &programmer):
  IntegerParameter(programmer, DELAY_FB_DC_CORR_MODE, "", 0x5CB1, 24, 20, 19, -2, 1, 0, "", "Dummy parameter for delay fb duty cycle correction (rev1 CDK only).", Parameter::IO_READ_WRITE, {}) {}
  virtual bool get(int &value, bool refresh = false)
  {
    value = _value;
    return true;
  }

  virtual bool set(const int &value)
  {
    _value = value;
    return true;
  }

  virtual ~TintinCDKDummyDelayFBDCCorrModeParameter() {}
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

bool TintinCDKCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice = _device;
  
  if (d.productID() == TINTIN_CDK_PRODUCT_UVC) 
  {
    _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(
      { {0x2D, 1}, {0x52, 1}, {0x54, 1}, {0x4B, 2}, {0x4E, 2}, {0x58, 3}, {0x5C, 3} },
      controlDevice));
    _streamer = Ptr<Streamer>(new UVCStreamer(controlDevice));
    
    _boardRevision[0] = _boardRevision[1] = 0;
  } 
  else 
  {
    USBIOPtr usbIO(new USBIO(controlDevice));

    _programmer = Ptr<RegisterProgrammer>(new VoxelUSBProgrammer(
      { {0x2D, 1}, {0x52, 1}, {0x54, 1}, {0x4B, 2}, {0x4E, 2}, {0x58, 3}, {0x5C, 3} },
      {
        {0x58, {0x08, 0x09, 0}},
        {0x5C, {0x08, 0x09, 0}},
        {0x4B, {0x0A, 0x0B, 0}},
        {0x4E, {0x0A, 0x0B, 0}},
        {0x2D, {0x04, 0x03, 8}},
        {0x52, {0x04, 0x03, 8}},
        {0x54, {0x04, 0x03, 8}}
      }, usbIO, controlDevice));
    _streamer = Ptr<Streamer>(new USBBulkStreamer(usbIO, controlDevice, 0x82));
    
    uint16_t length = 2;
    if(!usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, TINTIN_CDK_USBIO_BOARD_REVISION, 0, 0, _boardRevision, length))
    {
      logger(LOG_ERROR) << "TintinCDKCamera: Failed to get board revision" << std::endl;
      return false;
    }
    
    logger(LOG_INFO) << "TintinCDKCamera: Board revision = " << (int)_boardRevision[0] << "." << (int)_boardRevision[1] << std::endl;
    
    // Initialize serializer block
    configFile.setHardwareConfigSerializer(new HardwareSerializer(usbIO, REQUEST_EEPROM_DATA, REQUEST_EEPROM_SIZE));
  }
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  /* TPL0102 - remove non-volatile access */

  if (!_programmer->writeRegister(0x5410, 0xC0))
    return false;

  if(!_addParameters({
    ParameterPtr(new TintinCDKMixVoltageParameter(*_programmer)),
    ParameterPtr(new TintinCDKIllumVrefParameter(*_programmer)),
    ParameterPtr(new TintinCDKMainCurrentParameter(*_programmer)),
    ParameterPtr(new TintinCDKIllumCurrentParameter(*_programmer)),
    ParameterPtr(new TintinCDKIlluminationPowerParameter(*_programmer)),
    ParameterPtr(new TintinCDKIlluminationPowerPercentParameter(*this, *_programmer)),
    ParameterPtr(new TintinCDKModulationFrequencyParameter(*this, *_programmer, MOD_FREQ1, VCO_FREQ1, MOD_PS1)),
    ParameterPtr(new TintinCDKModulationFrequencyParameter(*this, *_programmer, MOD_FREQ2, VCO_FREQ2, MOD_PS2)),
    ParameterPtr(new TintinCDKUnambiguousRangeParameter(*this, *_programmer))
    }))
  {
    return false;
  }

  if ((int) _boardRevision[0] == 1) {
    if (!_addParameters({ParameterPtr(new TintinCDKPVDDParameterRev1(*_programmer)),}))
      return false;
  } else {
    if (!_addParameters({ParameterPtr(new TintinCDKPVDDParameterRev2(*_programmer)),}))
      return false;
  }

  /* INA226 initializations: note byteswapped for now */
  if (!_programmer->writeRegister(0x4B00, 0x274F) ||
    !_programmer->writeRegister(0x4E00, 0x274F) ||
    !_programmer->writeRegister(0x4B05, 0x4807) ||
    !_programmer->writeRegister(0x4E05, 0x4F1B)
  )
    return false;
    
  if(!ToFTintinCamera::_init())
    return false;
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_COMMON_PHASE_OFFSET];
    Vector<String> params = {ILLUM_POWER_PERCENTAGE, DELAY_FB_CORR_MODE, DELAY_FB_DC_CORR_MODE};
    calibInfo.definingParameters.insert(calibInfo.definingParameters.end(), params.begin(), params.end());
  }
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_TEMPERATURE];
    Vector<String> params = {ILLUM_POWER_PERCENTAGE};
    calibInfo.definingParameters.insert(calibInfo.definingParameters.end(), params.begin(), params.end());
  }
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_NON_LINEARITY];
    Vector<String> params = {ILLUM_POWER_PERCENTAGE};
    calibInfo.definingParameters.insert(calibInfo.definingParameters.end(), params.begin(), params.end());
  }
  
  Ptr<UnsignedIntegerParameter> pixCntMax = std::dynamic_pointer_cast<UnsignedIntegerParameter>(getParam(PIX_CNT_MAX));
  
  if(pixCntMax)
    pixCntMax->setLowerLimit(50000);
  
  Ptr<UnsignedIntegerParameter> intgDutyCycle = std::dynamic_pointer_cast<UnsignedIntegerParameter>(getParam(INTG_DUTY_CYCLE));

  if (intgDutyCycle)
    intgDutyCycle->setUpperLimit(20);

  Ptr<UnsignedIntegerParameter> intgTime = std::dynamic_pointer_cast<UnsignedIntegerParameter>(getParam(INTG_TIME));

  if (intgTime)
    intgTime->setUpperLimit(31);

  _parameters.erase(LUMPED_DEAD_TIME);
  _parameters.erase(ILLUM_DC_CORR);
  _parameters.erase(ILLUM_DC_CORR_DIR);

  /*
   * On the first revision of boards, these two parameters should not be set;
   * but we would like to keep a single set of calibration conf files for
   * both revisions of the board.
   * So use dummy parameters for now that mask the register writes till we find
   * a more elegant solution.
   */
  if ((int) _boardRevision[0] == 1) {
    _parameters.erase(DELAY_FB_CORR_MODE);
    _parameters.erase(DELAY_FB_DC_CORR_MODE);
    if (!_addParameters({ParameterPtr(new TintinCDKDummyDelayFBCorrModeParameter(*_programmer)),}))
      return false;
    if (!_addParameters({ParameterPtr(new TintinCDKDummyDelayFBDCCorrModeParameter(*_programmer)),}))
      return false;
    if (!set(DELAY_FB_COEFF1, 0U))
      return false;
  } else {
    if (
      !set(DELAY_FB_CORR_MODE, 1) ||
      !set(DELAY_FB_DC_CORR_MODE, 1))
      return false;
  }
  
  if (!set(COMP_VREF, 1405U))
    return false;
  
  if (!set(TILLUM_SLAVE_ADDR, 0x72U))
    return false;

  if ((d.productID() == TINTIN_CDK_PRODUCT_BULK) && !set(BLK_HEADER_EN, false))
    return false;

  return true;
}

bool TintinCDKCamera::_initStartParams()
{
  FrameSize s;
  USBDevice &d = (USBDevice &)*_device;

  if (!_getFrameSize(s) || !_setStreamerFrameSize(s))
    return false;

  if(!ToFTintinCamera::_initStartParams())
    return false;

  return true;
}



bool TintinCDKCamera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool TintinCDKCamera::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *uvcStreamer = dynamic_cast<UVCStreamer *>(&*_streamer);
  USBBulkStreamer *bulkStreamer = dynamic_cast<USBBulkStreamer *>(&*_streamer);
  USBDevice &d = (USBDevice &)*_device;
  
  VideoMode m;
  m.frameSize = s;
  
  int bytesPerPixel;
  int quadCount;
  ToFFrameType frameType;
  
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel) || !_get(QUAD_CNT_MAX, quadCount) || !_getToFFrameType(frameType))
  {
    logger(LOG_ERROR) << "TintinCDKCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  if(!_getFrameRate(m.frameRate))
  {
    logger(LOG_ERROR) << "TintinCDKCamera: Could not get current frame rate" << std::endl;
    return false;
  }
  
  if ((d.productID() == TINTIN_CDK_PRODUCT_UVC)) 
  {
    if(bytesPerPixel == 4)
      m.frameSize.width *= 2;
    if (!uvcStreamer) 
    {
      logger(LOG_ERROR) << "TintinCDKCamera: Streamer is not of type UVC" << std::endl;
      return false;
    }
    
    if(!uvcStreamer->setVideoMode(m)) 
    {
      logger(LOG_ERROR) << "TintinCDKCamera: Could not set video mode for UVC" << std::endl;
      return false;
    }
  } 
  else if ((d.productID() == TINTIN_CDK_PRODUCT_BULK)) 
  {
    if (!bulkStreamer) 
    {
      logger(LOG_ERROR) << "TintinCDKCamera: Streamer is not of type Bulk" << std::endl;
      return false;
    }
  
    uint scaleFactor = bytesPerPixel;
    if (frameType == ToF_QUAD)
      scaleFactor = quadCount * 2;

    if (!bulkStreamer->setBufferSize(s.width * s.height * scaleFactor)) 
    {
      logger(LOG_ERROR) << "TintinCDKCamera: Could not set buffer size for bulk transfer" << std::endl;
      return false;
    }
  }
  
  return true;
}

bool TintinCDKCamera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  USBDevice &d = (USBDevice &)*_device;

  if (d.productID() == TINTIN_CDK_PRODUCT_UVC) 
  {
    supportedVideoModes = Vector<SupportedVideoMode> {
      SupportedVideoMode(320,240,25,1,4),
      SupportedVideoMode(160,240,50,1,4),
      SupportedVideoMode(160,120,100,1,4),
      SupportedVideoMode(80,120,200,1,4),
      SupportedVideoMode(80,60,400,1,4),
      SupportedVideoMode(320,240,50,1,2),
      SupportedVideoMode(320,120,100,1,2),
      SupportedVideoMode(160,120,200,1,2),
      SupportedVideoMode(160,60,400,1,2),
      SupportedVideoMode(80,60,400,1,2),
    };
  } 
  else 
  {
    supportedVideoModes.clear();
  }

  return true;
}

bool TintinCDKCamera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  int bytesPerPixel;
  USBDevice &d = (USBDevice &)*_device;
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "TintinCDKCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  videoMode.frameSize.width = 320;
  videoMode.frameSize.height = 240;
  videoMode.frameRate.denominator = 1;
  if (d.productID() == TINTIN_CDK_PRODUCT_UVC) {
    videoMode.frameRate.numerator = (bytesPerPixel == 4)?25:50;
  } else {
    videoMode.frameRate.numerator = 60;
  }

  return true;
}

bool TintinCDKCamera::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  int opClockFrequency, bytesPerPixel;
  
  if(!_get(OP_CLK_FREQ, opClockFrequency) || !_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "TintinCDKCamera: Could not get " << OP_CLK_FREQ << " or " << PIXEL_DATA_SIZE << std::endl;
    return false;
  }
  
  opClockFrequency = 24/(1 << opClockFrequency);
  
  uint numerator = opClockFrequency*1000000,
  denominator = bytesPerPixel*forFrameSize.width*forFrameSize.height;
  
  uint g = gcd(numerator, denominator);
  
  frameRate.numerator = 0.8*numerator/g; // 90% of maximum
  frameRate.denominator = denominator/g;
  
  return true;
  
}
  
}
}