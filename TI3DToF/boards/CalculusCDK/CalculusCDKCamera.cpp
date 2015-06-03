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
    if(value > 0x80U)
      return (value - 0x80U)*25 + 500;
    else
      return 500;
  }
  
  virtual uint32_t _toRawValue(uint value) const
  {
    if(value > 500)
      return (value - 500)/25 + 0x80U;
    else
      return 0x80U;
  }
  
public:
  CalculusCDKMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 500, 2075, 1500, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~CalculusCDKMixVoltageParameter() {}
};


class CalculusCDKPVDDVoltageParameter: public UnsignedIntegerParameter
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
  CalculusCDKPVDDVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, PIXELVDD_VOLTAGE, "mV", 0x2D0E, 8, 7, 0, 500, 3650, 3300, "PixelVDD voltage", 
                           "Bias Voltage applied to the pixel array", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~CalculusCDKPVDDVoltageParameter() {}
};


bool CalculusCDKCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  controlDevice = _device;
  
  _usbIO = USBIOPtr(new USBIO(controlDevice));
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelUSBProgrammer(
    { {0x58, 3}, {0x2D, 1} },
    { 
      {0x58, {0x08, 0x09, 0}}, 
      {0x2D, {0x04, 0x03, 8}}, 
    }, _usbIO,
    controlDevice));
  
  _streamer = Ptr<Streamer>(new USBBulkStreamer(_usbIO, controlDevice, 0x82));

  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
#if 0
  if(!_addParameters({
    ParameterPtr(new CalculusCDKMixVoltageParameter(*_pmicProgrammer)),
    ParameterPtr(new CalculusCDKPVDDVoltageParameter(*_pmicProgrammer)),
    }))
  {
    return false;
  }
#endif
  
  if(!ToFCalculusCamera::_init())
    return false;

  // PVDD 3.0V
  _programmer->writeRegister(0x2D0E, 0xA8);
  
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
  
  if(
    // Enable Slave Fifo, reqCode 0x12
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x12, 0x00, 0x00) ||
    // Set VD Counter to 1300 for now
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x13, 1300, 0x00) ||
    // Enable FSC
    !_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x10, 0x0001, 0x00))
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
  supportedVideoModes = Vector<SupportedVideoMode> {
    SupportedVideoMode(80,60,30,1,4),
  };
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