/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelDCamera.h"
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
  
VoxelDCamera::VoxelDCamera(Voxel::DevicePtr device): ToFTintinCamera("VoxelDCamera", device)
{
  _init();
}

class VoxelDMixVoltageParameter: public UnsignedIntegerParameter
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
  VoxelDMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 500, 2075, 1500, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~VoxelDMixVoltageParameter() {}
};


class VoxelDIlluminationVoltageParameter: public UnsignedIntegerParameter
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
  VoxelDIlluminationVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_VOLTAGE, "mV", 0x2D0E, 8, 7, 0, 500, 3650, 1500, "Illumination voltage", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~VoxelDIlluminationVoltageParameter() {}
};


bool VoxelDCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  if(d.productID() == VOXEL_D_PRODUCT_ID1)
  {
    Vector<DevicePtr> devices = DeviceScanner::scan();
    
    uint repeatCount = 0;
    for(auto &d1: devices)
    {
      if(d1->interfaceID() != Device::USB)
        continue;
      
      USBDevice &usbd = (USBDevice &)*d1;
      
      if(usbd.vendorID() == VOXEL_D_VENDOR_ID && usbd.productID() == VOXEL_D_PRODUCT_ID2 && 
        (d.serialNumber().size() == 0 || d.serialNumber() == usbd.serialNumber()))
        repeatCount++;
    }
    
    
    _downloader = Ptr<Downloader>(new USBDownloader(_device));
    if(!_downloader->download(configFile.get("core", "fw"))) // TODO: This needs to come from a configuration
    {
      logger(LOG_ERROR) << "VoxelDCamera: Firmware download failed" << std::endl;
      return false;
    }
    else 
      std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait for device to get ready after loading firmware
      
    controlDevice = DevicePtr(new USBDevice(VOXEL_D_VENDOR_ID, VOXEL_D_PRODUCT_ID2, _device->serialNumber(), 
                                            _device->channelID(), _device->description(), _device->serialIndex(), repeatCount > 0));
    _device = controlDevice;
    _makeID();
  }
  else
    controlDevice = _device;
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(
    { {0x2D, 1}, {0x58, 3}, {0x5C, 3} },
    controlDevice));
  _streamer = Ptr<Streamer>(new UVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if(!_addParameters({
    ParameterPtr(new VoxelDMixVoltageParameter(*_programmer)),
    ParameterPtr(new VoxelDIlluminationVoltageParameter(*_programmer)),
    }))
  {
    return false;
  }
  
  if(!ToFTintinCamera::_init())
    return false;
  
  
  return true;
}

bool VoxelDCamera::_initStartParams()
{
  /*
   * The first two magic register writes are to invert the illum_en polarity
   * They are required only for the first set of boards, and should not be done
   * for the later revisions.
   * TODO: Add these to named parameters instead of hardcoded register values
   */
  return 
    _programmer->writeRegister(0x58A9, 0x800202) &&
    _programmer->writeRegister(0x58AD, 0x000181) &&
    set(ILLUM_VOLTAGE, 1800U) && 
    ToFTintinCamera::_initStartParams() &&
    set(MOD_FREQ1, 24.0f) &&
    set(CONFIDENCE_THRESHOLD, 2U);
}



bool VoxelDCamera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool VoxelDCamera::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *streamer = dynamic_cast<UVCStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "VoxelDCamera: Streamer is not of type UVC" << std::endl;
    return false;
  }
  
  VideoMode m;
  m.frameSize = s;
  
  int bytesPerPixel;
  
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "VoxelDCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  if(bytesPerPixel == 4)
    m.frameSize.width *= 2;
  
  if(!_getFrameRate(m.frameRate))
  {
    logger(LOG_ERROR) << "VoxelDCamera: Could not get current frame rate" << std::endl;
    return false;
  }
  
  if(!streamer->setVideoMode(m))
  {
    logger(LOG_ERROR) << "VoxelDCamera: Could not set video mode for UVC" << std::endl;
    return false;
  }
  
  return true;
}

bool VoxelDCamera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
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
  return true;
}

bool VoxelDCamera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  int bytesPerPixel;
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "VoxelDCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  videoMode.frameSize.width = 320;
  videoMode.frameSize.height = 240;
  videoMode.frameRate.denominator = 1;
  videoMode.frameRate.numerator = (bytesPerPixel == 4)?25:50;
  return true;
}


  
}
}
