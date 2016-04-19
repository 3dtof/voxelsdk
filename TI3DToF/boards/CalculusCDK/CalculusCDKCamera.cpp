/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CalculusCDKCamera.h"
#include "VoxelUSBProgrammer.h"
#include <Logger.h>
#include <USBBulkStreamer.h>

#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "USBSystem.h"

#define REQUEST_EEPROM_SIZE 0x31
#define REQUEST_EEPROM_DATA 0x33

namespace Voxel
{
  
namespace TI
{
  
CalculusCDKCamera::CalculusCDKCamera(Voxel::DevicePtr device): ToFCalculusCamera("CalculusCDKCamera", device)
{
  _init();
}

class CalculusCDKMixVoltageParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    if(value > 0x1AU)
      return 2000;
    else if (value == 0)
      return 800;
    else
      return (value)*50 + 750;
  }
  
  virtual uint32_t _toRawValue(uint value) const
  {
    if(value < 800)
      return 0x00U;
    else
      return (value - 750)/50;
  }
  
public:
  CalculusCDKMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x6023, 8, 7, 0, 800, 2000, 1800, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~CalculusCDKMixVoltageParameter() {}
};


class CalculusCDKIlluminationPowerParameter: public UnsignedIntegerParameter
{
protected:
  virtual bool get(uint &value, bool refresh = false)
  {
    value = uint(300);
    return true;
  }

  virtual bool set(const uint &value)
  {
    return true;
  }

#if 0
  virtual uint _fromRawValue(uint32_t value) const
  {
    return uint(299);
  }

  virtual uint32_t _toRawValue(uint value) const
  {
    return uint32_t(299);
  }
#endif

public:
  CalculusCDKIlluminationPowerParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER, "mW", 0, 0, 0, 0, 0, 300, 300, "Illumination Power", "These power numbers are approximate.", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~CalculusCDKIlluminationPowerParameter() {}
};

class CalculusCDKIlluminationPowerPercentParameter: public UnsignedIntegerParameter
{
protected:
  CalculusCDKCamera &_depthCamera;

public:
  CalculusCDKIlluminationPowerPercentParameter(CalculusCDKCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER_PERCENTAGE, "%", 0, 0, 0, 0, 0, 100, 100, "Illumination Power",
                          "These power numbers are approximate", Parameter::IO_READ_WRITE, {}),
                          _depthCamera(depthCamera)
  {}

  virtual bool get(uint &value, bool refresh = false)
  {
    uint v;
    value = (uint)((100.0f));
    return true;
  }

  virtual bool set(const uint &value)
  {
    return true;
  }

  virtual ~CalculusCDKIlluminationPowerPercentParameter() {}
};

class CalculusCDKIllumCurrentParameter: public UnsignedIntegerParameter
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
  CalculusCDKIllumCurrentParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, "illum_current", "mA", 0x4E04, 8, 7, 0, 0, 8000, 0000, "Illumination Current", 
                           "This is the current on the illumination 5V rail.", Parameter::IO_READ_ONLY, {})
  {}
  
  virtual ~CalculusCDKIllumCurrentParameter() {}
};

class CalculusCDKMainCurrentParameter: public UnsignedIntegerParameter
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
  CalculusCDKMainCurrentParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, "main_current", "mA", 0x4B04, 8, 7, 0, 0, 2000, 0000, "Main Board Current", 
                           "This is the current drawn by the main board.", Parameter::IO_READ_ONLY, {})
  {}
  
  virtual ~CalculusCDKMainCurrentParameter() {}
};


bool CalculusCDKCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  controlDevice = _device;
  
  _usbIO = USBIOPtr(new USBIO(controlDevice));
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelUSBProgrammer(
    { {0x58, 3}, {0x60, 1}, {0x4B, 2}, {0x4E, 2} },
    { 
      {0x58, {0x08, 0x09, 0}},
      {0x4B, {0x0A, 0x0B, 0}},
      {0x4E, {0x0A, 0x0B, 0}}, 
      {0x60, {0x0C, 0x0D, 0}}, 
    }, _usbIO,
    controlDevice));
  
  _streamer = Ptr<Streamer>(new USBBulkStreamer(_usbIO, controlDevice, 0x82));

  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if (!_addParameters({
    ParameterPtr(new CalculusCDKMixVoltageParameter(*_programmer)),
    ParameterPtr(new CalculusCDKMainCurrentParameter(*_programmer)),
    ParameterPtr(new CalculusCDKIllumCurrentParameter(*_programmer)),
    ParameterPtr(new CalculusCDKIlluminationPowerParameter(*_programmer)),
    ParameterPtr(new CalculusCDKIlluminationPowerPercentParameter(*this, *_programmer)),
    ParameterPtr(new CalculusModulationFrequencyParameter(*this, *_programmer, 12, 37, 18)),
    ParameterPtr(new CalculusUnambiguousRangeParameter(*this, *_programmer, 4, 50, 5)),
    }))
  {
    return false;
  }
  
  /* INA226 initializations: note byteswapped for now */
  if (!_programmer->writeRegister(0x4B00, 0x274F) ||
    !_programmer->writeRegister(0x4E00, 0x274F) ||
    !_programmer->writeRegister(0x4B05, 0x4807) ||
    !_programmer->writeRegister(0x4E05, 0x4F1B)
  )
    return false;
    
  // Initialize serializer block
  configFile.setHardwareConfigSerializer(new HardwareSerializer(_usbIO, REQUEST_EEPROM_DATA, REQUEST_EEPROM_SIZE));

  if(!ToFCalculusCamera::_init())
    return false;
  
  FrameSize s;
  
  if(!getFrameSize(s)) 
  {
    logger(LOG_ERROR) << "CalculusCDKCamera: Unable to get frame size" << std::endl;
    return false;
  }
  
  if(!_setStreamerFrameSize(s))
  {
    logger(LOG_ERROR) << "CalculusCDKCamera: Unable to set frame size for USBBulkStreamer" << std::endl;
    return false;
  }

  return true;
}

bool CalculusCDKCamera::_initStartParams()
{
  if (!ToFCalculusCamera::_initStartParams())
    return false;
  
  VoxelUSBProgrammer *programmer = dynamic_cast<VoxelUSBProgrammer *>(&*_programmer);
  
  if(!programmer)
  {
    logger(LOG_ERROR) << "CalculusCDKCamera: Streamer is not of type VoxelUSBProgrammer" << std::endl;
    return false;
  }
  
  uint16_t length = 0;
  
  if(
    // Enable Slave Fifo, reqCode 0x12
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x12, 0x00, 0x00, 0, length) ||
    // Set VD Counter to 1300 for now
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x13, 1300, 0x00, 0, length) ||
    // Enable FSC
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x10, 0x0001, 0x00, 0, length))
    return false;

  return true;
}

bool CalculusCDKCamera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool CalculusCDKCamera::_setStreamerFrameSize(const FrameSize &s)
{
  USBBulkStreamer *streamer = dynamic_cast<USBBulkStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "CalculusCDKCamera: Streamer is not of type USBBulkStreamer" << std::endl;
    return false;
  }
  
  uint bytesPerPixel;
  
  if(!_getBytesPerPixel(bytesPerPixel))
  {
    logger(LOG_ERROR) << "CalculusCDKCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  return streamer->setBufferSize(s.width*s.height*bytesPerPixel);
}

bool CalculusCDKCamera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  supportedVideoModes.clear();
  return true;
}

bool CalculusCDKCamera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  videoMode.frameSize.width = 80;
  videoMode.frameSize.height = 60;
  videoMode.frameRate.denominator = 1;
  videoMode.frameRate.numerator = 100;
  return true;
}

bool CalculusCDKCamera::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  frameRate.numerator = 500; // To be later updated to use OP_CLK_FREQ
  frameRate.denominator = 1;
  return true;
}

  
}
}