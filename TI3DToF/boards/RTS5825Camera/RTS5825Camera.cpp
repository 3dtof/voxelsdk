/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 */

#include "RTS5825Camera.h"
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
  
RTS5825Camera::RTS5825Camera(Voxel::DevicePtr device): ToFHaddockCamera("RTS5825Camera", device)
{
  _init();
}

class RTS5825CameraMixVoltageParameter: public UnsignedIntegerParameter
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
  RTS5825CameraMixVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, MIX_VOLTAGE, "mV", 0x2D05, 8, 7, 0, 500, 2075, 1500, "Mixing voltage", 
                           "Mixing voltage?", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~RTS5825CameraMixVoltageParameter() {}
};


class RTS5825CameraIlluminationVoltageParameter: public UnsignedIntegerParameter
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
  RTS5825CameraIlluminationVoltageParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_VOLTAGE, "mV", 0x2D0E, 8, 7, 0, 500, 1850, 1500, "Illumination voltage", 
                           "Voltage applied to the infra-red Illumination source", Parameter::IO_READ_WRITE, {})
  {}
  
  virtual ~RTS5825CameraIlluminationVoltageParameter() {}
};

class RTS5825CameraUVCStreamer : public UVCStreamer
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

  bool setVideoMode(const VideoMode &videoMode)
  {
    return true;
  }

  RTS5825CameraUVCStreamer(DevicePtr device) : UVCStreamer(device)
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

  virtual ~RTS5825CameraUVCStreamer()
  {
    if (isInitialized() && isRunning())
      stop();
    UVCStreamer::stop();
  }
};


bool RTS5825Camera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice = _device;
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(
    { {0x2D, 1}, {0x58, 3}, {0x5C, 3} },
    controlDevice));
  _streamer = Ptr<Streamer>(new RTS5825CameraUVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if(!_addParameters({
    ParameterPtr(new RTS5825CameraMixVoltageParameter(*_programmer)),
    ParameterPtr(new RTS5825CameraIlluminationVoltageParameter(*_programmer)),
    }))
  {
    return false;
  }
  
  if(!ToFHaddockCamera::_init())
    return false;
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_COMMON_PHASE_OFFSET];
    Vector<String> params = {ILLUM_VOLTAGE};
    calibInfo.definingParameters.insert(calibInfo.definingParameters.end(), params.begin(), params.end());
  }
  
  {
    CalibrationInformation &calibInfo = _getCalibrationInformationStructure()[ToF_CALIB_SECT_TEMPERATURE];
    Vector<String> params = {ILLUM_VOLTAGE};
    calibInfo.definingParameters.insert(calibInfo.definingParameters.end(), params.begin(), params.end());
  }
  
  return true;
}

bool RTS5825Camera::_initStartParams()
{
  return Voxel::TI::ToFCamera::_initStartParams();
}



bool RTS5825Camera::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool RTS5825Camera::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *streamer = dynamic_cast<UVCStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "RTS5825Camera: Streamer is not of type UVC" << std::endl;
    return false;
  }
  
  VideoMode m;
  m.frameSize = s;
  
  int bytesPerPixel = 4;
  
  if(bytesPerPixel == 4)
    m.frameSize.width *= 2;
  
  m.frameRate.denominator = 1;
  m.frameRate.numerator = 30;
  
  if(!streamer->setVideoMode(m))
  {
    logger(LOG_ERROR) << "RTS5825Camera: Could not set video mode for UVC" << std::endl;
    return false;
  }
  
  return true;
}

bool RTS5825Camera::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  supportedVideoModes = Vector<SupportedVideoMode> {
    SupportedVideoMode(320,240,30,1,4),
  };
  return true;
}

bool RTS5825Camera::_getMaximumVideoMode(VideoMode &videoMode) const
{
  int bytesPerPixel;
  if(!_get(PIXEL_DATA_SIZE, bytesPerPixel))
  {
    logger(LOG_ERROR) << "RTS5825Camera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  videoMode.frameSize.width = 320;
  videoMode.frameSize.height = 240;
  videoMode.frameRate.denominator = 1;
  videoMode.frameRate.numerator = (bytesPerPixel == 4)?30:30;
  return true;
}


  
}
}