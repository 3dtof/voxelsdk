/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#include "TintinCDKCameraUVC.h"
#include "VoxelXUProgrammer.h"
#include "VoxelUSBProgrammer.h"
#include <Logger.h>
#include <UVCStreamer.h>
#include <Parameter.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define REQUEST_EEPROM_SIZE 0x31
#define REQUEST_EEPROM_DATA 0x33

namespace Voxel
{

namespace TI
{

TintinCDKCameraUVC::TintinCDKCameraUVC(Voxel::DevicePtr device): ToFTintinCamera("TintinCDKCamera", device)
{
  _init();
}

bool TintinCDKCameraUVC::_init()
{
  USBDevice &d = (USBDevice &)*_device;

  DevicePtr controlDevice = _device;

  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(
   { {0x2D, 1}, {0x52, 1}, {0x54, 1}, {0x4B, 2}, {0x4E, 2}, {0x58, 3}, {0x5C, 3} },
   controlDevice));
  _streamer = Ptr<Streamer>(new UVCStreamer(controlDevice));


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

    if (!_addParameters({ParameterPtr(new TintinCDKPVDDParameterRev2(*_programmer)),}))
      return false;

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


  if (!set(COMP_VREF, 1405U))
    return false;

  if (!set(TILLUM_SLAVE_ADDR, 0x72U))
    return false;

  return true;
}

bool TintinCDKCameraUVC::_setStreamerFrameSize(const FrameSize &s)
{
  UVCStreamer *uvcStreamer = dynamic_cast<UVCStreamer *>(&*_streamer);
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

  return true;
}

bool TintinCDKCameraUVC::_initStartParams()
{
  FrameSize s;
  USBDevice &d = (USBDevice &)*_device;

  if (!_getFrameSize(s) || !_setStreamerFrameSize(s))
    return false;

  if(!ToFTintinCamera::_initStartParams())
    return false;

  return true;
}

bool TintinCDKCameraUVC::_getFieldOfView(float &fovHalfAngle) const
{
  fovHalfAngle = (87/2.0f)*(M_PI/180.0f);
  return true;
}

bool TintinCDKCameraUVC::_getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  USBDevice &d = (USBDevice &)*_device;

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

bool TintinCDKCameraUVC::_getMaximumVideoMode(VideoMode &videoMode) const
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
  videoMode.frameRate.numerator = (bytesPerPixel == 4)?25:50;


  return true;
}

bool TintinCDKCameraUVC::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
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
