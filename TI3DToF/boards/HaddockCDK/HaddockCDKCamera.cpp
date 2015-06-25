/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "HaddockCDKCamera.h"
#include "VoxelUSBProgrammer.h"
#include <Logger.h>
#include <USBBulkStreamer.h>

#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace Voxel
{
  
namespace TI
{
  
HaddockCDKCamera::HaddockCDKCamera(Voxel::DevicePtr device): ToFHaddockCamera("HaddockCDKCamera", device)
{
  _init();
}

class HaddockCDKMixVoltageParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    return uint((value)*2.319 + 1448.64);
  }
  
  virtual uint32_t _toRawValue(uint value) const
  {
    return uint32_t((value - 1448.64)/2.319);
  }
  
public:
  HaddockCDKMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2811, 8, 7, 0, 1449, 2200, 1800, "Mixing voltage", 
                           "Mixing voltage", Parameter::IO_WRITE_ONLY, {})
  {}

  
  virtual ~HaddockCDKMixVoltageParameter() {}
};


class HaddockCDKIlluminationPowerPercentageParameter: public UnsignedIntegerParameter
{
  HaddockCDKCamera &_depthCamera;
public:
  HaddockCDKIlluminationPowerPercentageParameter(HaddockCDKCamera &depthCamera, RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER_PERCENTAGE, "%", 0, 0, 0, 0, 0, 95, 20, "Illumination Power", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_WRITE_ONLY, {}),
                           _depthCamera(depthCamera)
  {}
  
  virtual bool set(const uint &value)
  {
    return _depthCamera._set(ILLUM_POWER, value);
  }
  
  virtual bool get(uint& value, bool refresh = false)
  {
    return _depthCamera._get(ILLUM_POWER, value, refresh);
  }
  
  virtual ~HaddockCDKIlluminationPowerPercentageParameter() {}
};

class HaddockCDKIlluminationPowerParameter: public UnsignedIntegerParameter
{
protected:
  virtual uint _fromRawValue(uint32_t value) const
  {
    return uint(100*(1-(value/255.0)));
  }
  
  virtual uint32_t _toRawValue(uint value) const
  {
    return uint32_t(255*(1-(value*1.0/100.0)));
  }
  
  Ptr<HaddockCDKIlluminationPowerParameter> _other;
  
public:
  HaddockCDKIlluminationPowerParameter(RegisterProgrammer &programmer, const String &name, uint32_t address):
  UnsignedIntegerParameter(programmer, name, "%", address, 8, 7, 0, 0, 95, 20, "Illumination Power", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_WRITE_ONLY, {})
  {}
  
  void setOtherParameter(Ptr<HaddockCDKIlluminationPowerParameter> other) { _other = other; }
  
  virtual bool set(const uint &value)
  {
    return UnsignedIntegerParameter::set(value) && (!_other || _other->set(value));
  }
  
  virtual ~HaddockCDKIlluminationPowerParameter() {}
};


bool HaddockCDKCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  controlDevice = _device;
  
  USBIOPtr usbIO(new USBIO(controlDevice));
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelUSBProgrammer(
    { {0x58, 3}, {0x5C, 3}, {0x28, 1}, {0x2A, 1}, {0x2B, 1} },
    { 
      {0x58, {0xA4, 0xA4, 0}}, 
      {0x5C, {0xA4, 0xA4, 0}}, 
      {0x28, {0xA4, 0xA4, 0}}, 
      {0x2A, {0xA4, 0xA4, 0}}, 
      {0x2B, {0xA4, 0xA4, 0}}
    }, usbIO,
    controlDevice));
  _streamer = Ptr<Streamer>(new USBBulkStreamer(usbIO, controlDevice, 0x86));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  auto p1 = Ptr<HaddockCDKIlluminationPowerPercentageParameter>(new HaddockCDKIlluminationPowerPercentageParameter(*this, *_programmer));
  
  auto p21 = Ptr<HaddockCDKIlluminationPowerParameter>(new HaddockCDKIlluminationPowerParameter(*_programmer, ILLUM_POWER, 0x2A11)),
  p22 = Ptr<HaddockCDKIlluminationPowerParameter>(new HaddockCDKIlluminationPowerParameter(*_programmer, ILLUM_POWER2, 0x2B11));
  
  p21->setOtherParameter(p22);
  
  if(!_addParameters({
    ParameterPtr(new HaddockCDKMixVoltageParameter(*_programmer)),
    std::dynamic_pointer_cast<Parameter>(p1),
    std::dynamic_pointer_cast<Parameter>(p21)
    }))
    return false;
  
  if(!ToFHaddockCamera::_init())
    return false;
  
  FrameSize s;
  
  if(!getFrameSize(s)) 
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Unable to get frame size" << std::endl;
    return false;
  }
  
  if(!_setStreamerFrameSize(s))
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Unable to set frame size for USBBulkStreamer" << std::endl;
    return false;
  }
  
  return true;
}

bool HaddockCDKCamera::_initStartParams()
{
  if(!set(TG_EN, true) || 
     !set(FB_READY_EN, true) ||
     !set(CONFIDENCE_THRESHOLD, 1U))
    return false;
  
  FrameSize s;
  
  // Re-calculate stream size everytime on start of stream. This is necessary because stream size 
  // depends on parameters which could be changed in between...
  if(!_getFrameSize(s) || !_setStreamerFrameSize(s))
    return false;
  
  return ToFCamera::_initStartParams(); // skip ToFHaddockCamera::_initStartParams()
}

bool HaddockCDKCamera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool HaddockCDKCamera::_setStreamerFrameSize(const FrameSize &s)
{
  USBBulkStreamer *streamer = dynamic_cast<USBBulkStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Streamer is not of type USBBulkStreamer" << std::endl;
    return false;
  }
  
  int bytesPerPixel;
  int quadCount;
  ToFFrameType frameType;
  
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel) || !_get(QUAD_CNT_MAX, quadCount) || !_getToFFrameType(frameType))
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  uint scaleFactor = bytesPerPixel;
  
  if(frameType == ToF_QUAD)
    scaleFactor = quadCount*2;
  
  if(!streamer->setBufferSize(s.height*s.width*scaleFactor))
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Could not set video mode for USBBulkStreamer" << std::endl;
    return false;
  }
  
  return true;
}

bool HaddockCDKCamera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  supportedVideoModes.clear();
  return true;
}

bool HaddockCDKCamera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  videoMode.frameSize.width = 320;
  videoMode.frameSize.height = 240;
  videoMode.frameRate.denominator = 1;
  videoMode.frameRate.numerator = 30;
  return true;
}

bool HaddockCDKCamera::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  int opClockFrequency, bytesPerPixel;
  
  if(!_get(OP_CLK_FREQ, opClockFrequency) || !_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "HaddockCDKCamera: Could not get " << OP_CLK_FREQ << " or " << PIXEL_DATA_SIZE << std::endl;
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