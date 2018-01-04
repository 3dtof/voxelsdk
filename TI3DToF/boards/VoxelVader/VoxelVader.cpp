/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#include "VoxelVader.h"

#include "VoxelSPCAXUProgrammer.h"
#include <Logger.h>
#include <UVCStreamer.h>

#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define SP_SERIAL_NUMBER_SIZE 14 //14 characters is the maximum size for serial number
#define SP_SERIAL_NUMBER_SECTOR_ADDRESS 0x8000 //0x8000 is the address for the serial number sector
#define SP_SERIAL_NUMBER_ADDRESS 0x8000 //8004 is the acutal address, but reading from 8000. 
#define SP_BYTES_TO_UPDATE 4096 //SP Flash memory requirement of 4 kB to be updated

namespace Voxel
{
  
namespace TI
{
  
VoxelVader::VoxelVader(Voxel::DevicePtr device): ToFCalculusCamera("VoxelVader", device)
{
  _init();
}

class VoxelVaderIlluminationPowerParameter: public UnsignedIntegerParameter
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

public:
  VoxelVaderIlluminationPowerParameter(RegisterProgrammer &programmer):
  UnsignedIntegerParameter(programmer, ILLUM_POWER, "mW", 0, 0, 0, 0, 0, 300, 300, "Illumination Power", "These power numbers are approximate.", Parameter::IO_READ_WRITE, {})
  {}

  virtual ~VoxelVaderIlluminationPowerParameter() {}
};

class VoxelVaderIlluminationPowerPercentParameter: public UnsignedIntegerParameter
{
protected:
  VoxelVader &_depthCamera;

public:
  VoxelVaderIlluminationPowerPercentParameter(VoxelVader &depthCamera, RegisterProgrammer &programmer):
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

  virtual ~VoxelVaderIlluminationPowerPercentParameter() {}
};

class VoxelVaderUVCStreamer : public UVCStreamer
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

  VoxelVaderUVCStreamer(DevicePtr device) : UVCStreamer(device)
  {
    VideoMode m;
    m.frameSize.width = 80*2;
    m.frameSize.height = 60;
    m.frameRate.numerator = 240;
    m.frameRate.denominator = 1;
    UVCStreamer::setVideoMode(m);
    UVCStreamer::start();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  virtual ~VoxelVaderUVCStreamer()
  {
    if (isInitialized() && isRunning())
      stop();
    UVCStreamer::stop();
  }
};


bool VoxelVader::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice = _device;

  _xu = UVCXUPtr(new UVCXU(_device, _XU_ID, 1));
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelSPCAXUProgrammer(
    { {0x58, 3} ,{0x60, 1} } ,
    controlDevice));
  _streamer = Ptr<Streamer>(new VoxelVaderUVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;

  
  if (!_addParameters({
      ParameterPtr(new CalculusModulationFrequencyParameter(*this, *_programmer, 12, 37, 18)),
      ParameterPtr(new CalculusUnambiguousRangeParameter(*this, *_programmer, 4, 50, 5)),
      }))
    {
      return false;
    }

  if(!_addParameters({
  ParameterPtr(new VoxelVaderIlluminationPowerPercentParameter(*this, *_programmer)),
  ParameterPtr(new VoxelVaderIlluminationPowerParameter(*_programmer)),
    }))
  {
    return false;
  }
  
  if(!ToFCalculusCamera::_init())
    return false;
  
  FrameSize s;

  if(!getFrameSize(s))
  {
    logger(LOG_ERROR) << "VoxelVader: Unable to get frame size" << std::endl;
    return false;
  }

  logger(LOG_DEBUG) << "VoxelVader: Frame Size = " << s.height << " " << s.width << std::endl;

  if(!_setStreamerFrameSize(s))
  {
    logger(LOG_ERROR) << "VoxelVader: Unable to set frame size for UVCStreamer" << std::endl;
    return false;
  }
  else
    logger(LOG_INFO) << "Frame Size=" << s.height << "  " << s.width << std::endl;

  Ptr<UnsignedIntegerParameter> intgDutyCycle = std::dynamic_pointer_cast<UnsignedIntegerParameter>(getParam(INTG_DUTY_CYCLE));

  if (intgDutyCycle)
    intgDutyCycle->setUpperLimit(9);

  Ptr<UnsignedIntegerParameter> intgTime = std::dynamic_pointer_cast<UnsignedIntegerParameter>(getParam(INTG_TIME));

  if (intgTime)
    intgTime->setUpperLimit(14);

  
  return true;
}

bool VoxelVader::_initStartParams()
{
  return Voxel::TI::ToFCamera::_initStartParams();
}



bool VoxelVader::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool VoxelVader::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *streamer = dynamic_cast<UVCStreamer *>(&*_streamer);
  
  if(!streamer)
  {
    logger(LOG_ERROR) << "VoxelVader: Streamer is not of type UVC" << std::endl;
    return false;
  }
  
  VideoMode m;
  m.frameSize = s;
  
  int bytesPerPixel = 4;
  
  if(bytesPerPixel == 4)
    m.frameSize.width *= 2;
  
  m.frameRate.denominator = 1;
  m.frameRate.numerator = 240;
  
  if(!streamer->setVideoMode(m))
  {
    logger(LOG_ERROR) << "VoxelVader: Could not set video mode for UVC" << std::endl;
    return false;
  }
  
  return true;
}

bool VoxelVader::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  supportedVideoModes = Vector<SupportedVideoMode> {
    SupportedVideoMode(80,60,240,1,4),
  };
  return true;
}

bool VoxelVader::_getMaximumVideoMode(VideoMode &videoMode) const
{
  int bytesPerPixel=4;
  
  videoMode.frameSize.width = 80;
  videoMode.frameSize.height = 60;
  videoMode.frameRate.denominator = 1;
  videoMode.frameRate.numerator = 240;
  return true;
}

bool VoxelVader::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  frameRate.numerator = 240; // To be later updated to use OP_CLK_FREQ
  frameRate.denominator = 1;
  return true;
}

bool VoxelVader::setSerialNumber(const String &serialNumber)
{
  if(serialNumber.size() > SP_SERIAL_NUMBER_SIZE)
  {
    logger(LOG_ERROR) << "VoxelVader: Please specify serial number with at most '" << SP_SERIAL_NUMBER_SIZE << "' bytes." << std::endl;
    return false;
  }
    
    uint16_t length = serialNumber.size();
    uint8_t buffer[SP_BYTES_TO_UPDATE];
    uint32_t remaining = SP_BYTES_TO_UPDATE;
    uint8_t data[3]; // for the start address
    uint32_t offset = 0;
    while (remaining)
    {
      int size = remaining%32?remaining%32:32;
      data[0] = (SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) & 0xFF;
      data[1] = ((SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) >> 8) & 0xFF;
      data[2] = ((SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) >> 16) & 0xFF; 
      
      if (!_xu->setControl(CONTROL_READ_WRITE_REGISTER_START, sizeof(data), data))
      {
        logger(LOG_ERROR) << "VoxelVader: Could not initialize XU control." << std::endl;
        return false; 
      }

      if(!_xu->getControl(CONTROL_READ_WRITE_REGISTER_32, size, (buffer + offset)))
      {
        logger(LOG_ERROR) << "VoxelVader: Could not set serial number from depth camera. "<< std::endl;
        return false;
      }

      offset = offset + size;
      remaining = remaining - size;
    }
  
  for (auto i = 0; i<length;i++)
  {
    buffer[4+2*i] = (uint8_t)serialNumber[i];
    buffer[4+2*i+1] = 0x00;
  }

  if(length < SP_SERIAL_NUMBER_SIZE)
  {
    for(auto i = 2*length; i < 2*SP_SERIAL_NUMBER_SIZE; i++)
      buffer[4+i] = 0x00;
  }


  for (auto i = 2*SP_SERIAL_NUMBER_SIZE; i < SP_BYTES_TO_UPDATE - 4; i++)
    buffer[4+i] = 0xFF;

  data[0] = SP_SERIAL_NUMBER_SECTOR_ADDRESS & 0xff;
  data[1] = (SP_SERIAL_NUMBER_SECTOR_ADDRESS >> 8) & 0xff;
  data[2] = (SP_SERIAL_NUMBER_SECTOR_ADDRESS >> 16) & 0xff;

  if(!_xu->setControl(CONTROL_ERASE_DATA, sizeof(data), data))
  {
    logger(LOG_ERROR) << "VoxelVader: Cannot erase data before writing" << std::endl;
    return false;
  }

  remaining = SP_BYTES_TO_UPDATE; 
  offset = 0;
  while(remaining)
  {
   {
      int size = remaining%32 ? remaining%32:32;
      data[0] = (SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) & 0xFF;
      data[1] = ((SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) >> 8) & 0xFF;
      data[2] = ((SP_SERIAL_NUMBER_SECTOR_ADDRESS + offset) >> 16) & 0xFF; 
      
      if (!_xu->setControl(CONTROL_READ_WRITE_REGISTER_START, sizeof(data), data))
      {
        logger(LOG_ERROR) << "VoxelVader: Could not initialize XU control." << std::endl;
        return false; 
      }

      if(!_xu->setControl(CONTROL_READ_WRITE_REGISTER_32, size, (buffer + offset)))
      {
        logger(LOG_ERROR) << "VoxelVader: Could not write serial number to depth camera." <<  std::endl;
        return false;
      }

      offset = offset + size;
      remaining = remaining - size;
    }

    _device->setSerialNumber(serialNumber);
    return true;

  }

  return DepthCamera::setSerialNumber(serialNumber);
}

bool VoxelVader::getSerialNumber(String &serialNumber) const 
{ 
  uint8_t data[3];
  data[0] = (SP_SERIAL_NUMBER_ADDRESS) & 0xFF;
  data[1] = ((SP_SERIAL_NUMBER_ADDRESS) >> 8) & 0xFF;
  data[2] = ((SP_SERIAL_NUMBER_ADDRESS) >> 16) & 0xFF;   

  if (!_xu->setControl(CONTROL_READ_WRITE_REGISTER_START, sizeof(data), data))
      {
        logger(LOG_ERROR) << "VoxelVader: Could not initialize XU control." << std::endl;
        return false; 
      }

  int size = 32; 
  uint8_t buffer[32];
  if(!_xu->getControl(CONTROL_READ_WRITE_REGISTER_32, size, buffer))
  {
    logger(LOG_ERROR) << "VoxelVader: Could not read serial number from depth camera." << std::endl;
    return false;
  }

  char sn[SP_SERIAL_NUMBER_SIZE +1];

  for (auto i = 0; i<SP_SERIAL_NUMBER_SIZE; i++)
  {
    sn[i] = buffer[4+2*i];
  }  
  sn[SP_SERIAL_NUMBER_SIZE] = '\0';
  serialNumber = sn;

  return true;
}
  
}
}
