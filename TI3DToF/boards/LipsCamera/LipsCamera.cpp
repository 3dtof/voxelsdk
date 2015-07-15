/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "LipsCamera.h"
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
  
LipsCamera::LipsCamera(Voxel::DevicePtr device): ToFHaddockCamera("LipsCamera", device)
{
  _init();
}

class LipsMixVoltageParameter: public UnsignedIntegerParameter
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
  LipsMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 500, 2075, 1500, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~LipsMixVoltageParameter() {}
};


class LipsIlluminationVoltageParameter: public UnsignedIntegerParameter
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
  LipsIlluminationVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_VOLTAGE, "mV", 0x2D0E, 8, 7, 0, 500, 1850, 1500, "Illumination voltage", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~LipsIlluminationVoltageParameter() {}
};

class LipsCameraUVCStreamer : public UVCStreamer
{
protected:
  bool _isReallyRunning = false;
public:
  bool start()
  {
    return _isReallyRunning = true;
  }

  bool stop()
  {
    _isReallyRunning = false;
    return true;
  }

  bool isRunning()
  {
    return _isReallyRunning;
  }

  LipsCameraUVCStreamer(DevicePtr device) : UVCStreamer(device)
  {
    VideoMode m;
    m.frameSize.width = 320*2;
    m.frameSize.height = 240;
    m.frameRate.numerator = 25;
    m.frameRate.denominator = 1;
    UVCStreamer::setVideoMode(m);
    UVCStreamer::start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  virtual ~LipsCameraUVCStreamer()
  {
    if (isInitialized() && isRunning())
      stop();
    UVCStreamer::stop();
  }
};


bool LipsCamera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice = _device;
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(
    { {0x2D, 1}, {0x58, 3}, {0x5C, 3} },
    controlDevice));
  _streamer = Ptr<Streamer>(new LipsCameraUVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if(!_addParameters({
    ParameterPtr(new LipsMixVoltageParameter(*_programmer)),
    ParameterPtr(new LipsIlluminationVoltageParameter(*_programmer)),
    }))
  {
    return false;
  }
  
  if(!ToFHaddockCamera::_init())
    return false;
  
  
  return true;
}

bool LipsCamera::_initStartParams()
{
  if(!set(ILLUM_VOLTAGE, 1500U))
    return false;
  return 
  //set(ILLUM_VOLTAGE, 1500U) && 
  //set(MIX_VOLTAGE, 1500U) &&
  Voxel::TI::ToFCamera::_initStartParams();
}



bool LipsCamera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool LipsCamera::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *streamer = dynamic_cast<UVCStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "LipsCamera: Streamer is not of type UVC" << std::endl;
    return false;
  }
  
  VideoMode m;
  m.frameSize = s;
  
  int bytesPerPixel;
  
#if 0
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "LipsCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
#else
  bytesPerPixel = 4;
#endif
  
  if(bytesPerPixel == 4)
    m.frameSize.width *= 2;
  
#if 0
  if(!_getFrameRate(m.frameRate))
  {
    logger(LOG_ERROR) << "LipsCamera: Could not get current frame rate" << std::endl;
    return false;
  }
#endif
  m.frameRate.denominator = 1;
  m.frameRate.numerator = 25;
  
  if(!streamer->setVideoMode(m))
  {
    logger(LOG_ERROR) << "LipsCamera: Could not set video mode for UVC" << std::endl;
    return false;
  }
  
  return true;
}

bool LipsCamera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  supportedVideoModes = Vector<SupportedVideoMode> {
    SupportedVideoMode(320,240,25,1,4),
    // SupportedVideoMode(160,240,50,1,4),
    // SupportedVideoMode(160,120,100,1,4),
    // SupportedVideoMode(80,120,200,1,4),
    // SupportedVideoMode(80,60,400,1,4),
    // SupportedVideoMode(320,240,50,1,2),
    // SupportedVideoMode(320,120,100,1,2),
    // SupportedVideoMode(160,120,200,1,2),
    // SupportedVideoMode(160,60,400,1,2),
    // SupportedVideoMode(80,60,400,1,2),
  };
  return true;
}

bool LipsCamera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  int bytesPerPixel;
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "LipsCamera: Could not get current bytes per pixel" << std::endl;
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