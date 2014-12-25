/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Voxel14Camera.h"
#include "VoxelXUProgrammer.h"
#include <Logger.h>
#include <UVCStreamer.h>

#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>
namespace Voxel
{
  
namespace TI
{
  
Voxel14Camera::Voxel14Camera(Voxel::DevicePtr device): ToFHaddockCamera("Voxel14Camera", device)
{
  _init();
}

class MixVoltageParameter: public UnsignedIntegerParameter
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
  MixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 500, 2075, 1500, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~MixVoltageParameter() {}
};


class IlluminationVoltageParameter: public UnsignedIntegerParameter
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
  IlluminationVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_VOLTAGE, "mV", 0x2D0E, 8, 7, 0, 500, 3650, 1500, "Illumination voltage", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~IlluminationVoltageParameter() {}
};


bool Voxel14Camera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  if(d.productID() == VOXEL_14_PRODUCT_ID1)
  {
    _downloader = Ptr<Downloader>(new USBDownloader(_device));
    if(!_downloader->download("OPT9220_0v27.fw")) // TODO: This needs to come from a configuration
    {
      logger(LOG_ERROR) << "Voxel14Camera: Firmware download failed" << endl;
      return false;
    }
    else 
      std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait for device to get ready after loading firmware
      
    controlDevice = DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID2, _device->serialNumber()));
  }
  else
    controlDevice = _device;
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(controlDevice));
  _streamer = Ptr<Streamer>(new UVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if(!ToFHaddockCamera::_init())
    return false;
  
  _addParameters({
    ParameterPtr(new MixVoltageParameter(*_programmer)),
    ParameterPtr(new IlluminationVoltageParameter(*_programmer)),
  });
  
  return true;
}

bool Voxel14Camera::_initStartParams()
{
  return 
  set(ILLUM_VOLTAGE, 1500U) && 
  set(MIX_VOLTAGE, 1500U) &&
  Voxel::TI::ToFHaddockCamera::_initStartParams();
}



bool Voxel14Camera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

  
}
}