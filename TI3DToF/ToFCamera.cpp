/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

namespace Voxel
{
  
namespace TI
{
  
class IntegrationTimeParameter: public FloatParameter
{
  ToFCamera &_depthCamera;
public:
  IntegrationTimeParameter(ToFCamera &depthCamera, RegisterProgrammer &programmer):
  FloatParameter(programmer, INTG_TIME, "%", 0, 0, 0, 1, 0, 100, 0, "Integration time", 
                "Integration time as percentage of total cycle time", Parameter::IO_READ_WRITE, {INTG_DUTY_CYCLE}), _depthCamera(depthCamera) {}
                
  virtual bool get(float &value, bool refresh = false)
  {
    uint integrationDutyCycle;
    
    bool integrationDutyCycleSetFailed;
    
    if(!_depthCamera._get(INTG_DUTY_CYCLE, integrationDutyCycle, refresh) || 
      !_depthCamera._get(INTG_DUTY_CYCLE_SET_FAILED, integrationDutyCycleSetFailed, refresh)
      || integrationDutyCycleSetFailed)
      return false;
    
    
    float v = integrationDutyCycle*100/63;
    
    if(v > 100) v = 100;
    if(v < 0) v = 0;
    
    value = v;
    return true;
  }
  
  virtual bool set(const float &value)
  {
    if(!validate(value))
      return false;
    
    uint integrationDutyCycle = (value/100)*63;
    
    if(integrationDutyCycle > 63) integrationDutyCycle = 63;
    
    if(!_depthCamera._set(INTG_DUTY_CYCLE, integrationDutyCycle))
      return false;
    
    bool integrationDutyCycleSetFailed;
    
    if(!_depthCamera._get(INTG_DUTY_CYCLE_SET_FAILED, integrationDutyCycleSetFailed)
      || integrationDutyCycleSetFailed)
      return false;
    
    return true;
  }
};
  

ToFCamera::ToFCamera(const String &name, DevicePtr device): ToFCameraBase(name, device),
_tofFrameGenerator(new ToFFrameGenerator())
{
  _frameGenerators[0] = std::dynamic_pointer_cast<FrameGenerator>(_tofFrameGenerator);
  _tofDepthFrameGenerator->setProcessedFrameGenerator(_frameGenerators[0]);
}

  
bool ToFCamera::_init()
{
  if(!_addParameters({
    ParameterPtr(new IntegrationTimeParameter(*this, *_programmer))
  }))
    return false;
  
  if(!ToFCameraBase::_init())
  {
    return false;
  }
  
  return true;
}

bool ToFCamera::_getFrameRate(FrameRate &r) const
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, pixCount, sysClkFrequency;
  
  if(!_get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  if(!_get(PIX_CNT_MAX, pixCount) || !_get(QUAD_CNT_MAX, quadCount) || !_get(SUBFRAME_CNT_MAX, subFrameCount) || !_getSystemClockFrequency(sysClkFrequency))
    return false;
  
  uint numerator = sysClkFrequency*1000000,
  denominator = pixCount*quadCount*subFrameCount;
  
  uint g = gcd(numerator, denominator);
  
  r.numerator = numerator/g;
  r.denominator = denominator/g;
  return true;
}

bool ToFCamera::_setFrameRate(const FrameRate &r)
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, sysClkFrequency, pixCount;
  
  if(!_get(QUAD_CNT_MAX, quadCount) || !_get(SUBFRAME_CNT_MAX, subFrameCount) || !_getSystemClockFrequency(sysClkFrequency))
    return false;
  
  if(r.numerator == 0 || quadCount == 0 || subFrameCount == 0)
    return false;
  
  pixCount = (uint)((((uint64_t)r.denominator)*sysClkFrequency*1000000L)/(((uint64_t)quadCount)*subFrameCount*r.numerator));
  
  logger(LOG_DEBUG) << "ToFCamera: Setting " << PIX_CNT_MAX << " = " << pixCount << std::endl;
  
  if(!_set(PIX_CNT_MAX, pixCount) || !_get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  return true;
}

bool ToFCamera::_getFrameSize(Voxel::FrameSize &s) const
{
  uint binRowCount, binColumnCount;
  
  if(!_get(BIN_ROW_COUNT, binRowCount) || !_get(BIN_COLUMN_COUNT, binColumnCount))
    return false;
  
  s.width = binColumnCount;
  s.height = binRowCount;
  return true;
}

bool ToFCamera::_setFrameSize(const FrameSize &s)
{
  return _setFrameSize(s, true);
}

bool ToFCamera::_setFrameSize(const FrameSize &s, bool resetROI)
{
  if(isRunning())
  {
    logger(LOG_ERROR) << "ToFCamera: Cannot set frame size while the camera is streaming" << std::endl;
    return false;
  }
  
  RegionOfInterest roi;
  if(resetROI)
  {
    FrameSize maxFrameSize;
    
    if(!_getMaximumFrameSize(maxFrameSize))
    {
      logger(LOG_ERROR) << "ToFCamera: Could not get maximum frame size. Need that to reset ROI" << std::endl;
      return false;
    }
    
    roi.x = roi.y = 0;
    roi.width = maxFrameSize.width;
    roi.height = maxFrameSize.height;
    
    if(!_setROI(roi))
    {
      logger(LOG_ERROR) << "ToFCamera: Could not reset ROI" << std::endl;
      return false;
    }
  }
  else if(!_getROI(roi))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get current ROI, to set frame size" << std::endl;
    return false;
  }
  
  FrameSize toSet;
  
  toSet.width = (s.width <= roi.width)?s.width:roi.width;
  toSet.height = (s.height <= roi.height)?s.height:roi.height;
  
  Vector<SupportedVideoMode> supportedVideoModes;
  
  uint bytesPerPixel;
  
  if(!_getSupportedVideoModes(supportedVideoModes) || !_getBytesPerPixel(bytesPerPixel))
  {
    logger(LOG_ERROR) << "Could not get supported video modes or current bytes per pixel, to get nearest valid frame size" << std::endl;
    return false;
  }
  
  int maxScore = 0;
  IndexType index = -1;
  int area = toSet.height*toSet.width;
  
  if(supportedVideoModes.size() > 0)
  {
    for(IndexType i = 0; i < supportedVideoModes.size(); i++)
    {
      auto &a = supportedVideoModes[i];
      
      int scoreA = a.frameSize.width*a.frameSize.height;
      scoreA = (bytesPerPixel != a.bytesPerPixel || scoreA > area || a.frameSize.width > toSet.width || a.frameSize.height > toSet.height)?0:scoreA;
      
      if(scoreA > maxScore)
      {
        maxScore = scoreA;
        index = i;
      }
    }
    
    if(index < 0)
    {
      logger(LOG_ERROR) << "ToFCamera: No supported frame size exists close to the desired frame size. Could not set frame size." << std::endl;
      return false;
    }
    toSet = supportedVideoModes[index].frameSize;
  }
  
  uint rowsToMerge = (roi.height + toSet.height - 1)/toSet.height,
  columnsToMerge = (roi.width + toSet.width - 1)/toSet.width;
  
  if(!_setBinning(rowsToMerge, columnsToMerge, toSet))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set binning for required frame size" << std::endl;
    return false;
  }
  
  if(!_setStreamerFrameSize(toSet))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get streamer's frame size" << std::endl;
    return false;
  }
  
  return true;
}

bool ToFCamera::_getBytesPerPixel(uint &bpp) const
{
  int b;
  
  if(!_get(PIXEL_DATA_SIZE, b))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get current bytes per pixel" << std::endl;
    return false;
  }
  
  bpp = (uint)b;
  return true;
}

bool ToFCamera::_setBytesPerPixel(const uint &bpp)
{
  int b = (int)bpp;
  int dataArrangeMode = (b == 4)?2:0;
  
  if(!_set(PIXEL_DATA_SIZE, b) || !_set(OP_DATA_ARRANGE_MODE, dataArrangeMode))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set bytes per pixel or data arrange mode" << std::endl;
    return false;
  }
  
  return true;
}

bool ToFCamera::_getOpDataArrangeMode(int &dataArrangeMode) const
{
  int b;

  if(!get(OP_DATA_ARRANGE_MODE, b))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get data arrange mode" << std::endl;
    return false;
  }

  dataArrangeMode = b;
  return true;
}

bool ToFCamera::_getBinning(uint &rowsToMerge, uint &columnsToMerge) const
{
  bool binningEnabled;
  
  if(!get(BINNING_EN, binningEnabled))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get binning_en" << std::endl;
    return false;
  }
  
  if(!binningEnabled)
  {
    rowsToMerge = columnsToMerge = 1;
    return true;
  }
  
  if(!get(BIN_ROWS_TO_MERGE, rowsToMerge) || !get(BIN_COLS_TO_MERGE, columnsToMerge))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set binning related parameters" << std::endl;
    return false;
  }
  
  return true;
}

bool ToFCamera::_setBinning(uint rowsToMerge, uint columnsToMerge, const FrameSize &frameSize)
{
  if(!_set(BIN_ROWS_TO_MERGE, rowsToMerge) || !_set(BIN_COLS_TO_MERGE, columnsToMerge) ||
    !_set(BIN_ROW_COUNT, (uint)frameSize.height) || !_set(BIN_COLUMN_COUNT, (uint)frameSize.width)
     || !_set(BINNING_EN, true))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set binning related parameters" << std::endl;
    return false;
  }
  return true;
}

bool ToFCamera::_getAmplitudeNormalizingFactor(float &factor)
{
  factor = 1.0/(1 << 12);
  return true;
}

bool ToFCamera::_is16BitModeEnabled(bool &mode16Bit)
{
  return get(DEALIAS_16BIT_OP_ENABLE, mode16Bit);
}

bool ToFCamera::_processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput)
{
  RegionOfInterest roi;
  uint rowsToMerge, columnsToMerge;
  FrameSize maxFrameSize, frameSize;
  
  if(!getMaximumFrameSize(maxFrameSize) || !getFrameSize(frameSize) || !getROI(roi) || !_getBinning(rowsToMerge, columnsToMerge))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not get frame related parameters. Cannot convert raw data to ToF data" << std::endl;
    return false;
  }
  
  uint bytesPerPixel;
  int dataArrangeMode;
  uint quadCount;
  ToFFrameType type;
  
  bool dealiased16BitMode;
  if(!_getBytesPerPixel(bytesPerPixel) || !_getOpDataArrangeMode(dataArrangeMode) || !_getToFFrameType(type) ||
    !_is16BitModeEnabled(dealiased16BitMode) || !get(QUAD_CNT_MAX, quadCount))
  {
    logger(LOG_ERROR) << "ToFCamera: Failed to read " << PIXEL_DATA_SIZE << " or " 
      << OP_DATA_ARRANGE_MODE << " or " << type << std::endl;
    return false;
  }
  
  if(!_tofFrameGenerator->setParameters(configFile.get("calib", "phasecorrection"), bytesPerPixel, 
                                    dataArrangeMode, roi, maxFrameSize, frameSize, rowsToMerge, columnsToMerge, _isHistogramEnabled(),
                                    configFile.get("calib", "cross_talk_coeff"),
                                    type, quadCount, dealiased16BitMode
                                   ))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set parameters to ToFFrameGenerator" << std::endl;
    return false;
  }
  
  FramePtr p1 = std::dynamic_pointer_cast<Frame>(rawFrameInput);
  FramePtr p2 = std::dynamic_pointer_cast<Frame>(rawFrameOutput);
  
  bool ret = _tofFrameGenerator->generate(p1, p2);
  
  if(ret)
  {
    rawFrameOutput = std::dynamic_pointer_cast<RawFrame>(p2);
    return true;
  }
  else
    return false;
}

bool ToFCamera::_initStartParams()
{
  RegionOfInterest roi;
  uint rowsToMerge, columnsToMerge;
  
  if(!_getBinning(rowsToMerge, columnsToMerge) ||
    !getROI(roi))
    return false;
  
  if(!_pointCloudFrameGenerator->setParameters(
    roi.x, roi.y, roi.width, roi.height,
    rowsToMerge, columnsToMerge,
    configFile.getFloat("calib", "fx"), // fx
    configFile.getFloat("calib", "fy"), // fy
    configFile.getFloat("calib", "cx"),// cx
    configFile.getFloat("calib", "cy"),// cy
    configFile.getFloat("calib", "k1"),// k1
    configFile.getFloat("calib", "k2"),// k2
    configFile.getFloat("calib", "k3"),// k3
    configFile.getFloat("calib", "p1"),// p1
    configFile.getFloat("calib", "p2") // p2
  ))
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set parameters to PointCloudFrameGenerator" << std::endl;
    return false;
  }
  
  if(!_applyCalibrationParams())
  {
    logger(LOG_ERROR) << "ToFCamera: Could not set calibration parameters" << std::endl;
    return false;
  }
  
  return true;
}


bool ToFCamera::_getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  Vector<SupportedVideoMode> supportedVideoModes;
  
  uint bytesPerPixel;
  
  if(!_getSupportedVideoModes(supportedVideoModes) || !_getBytesPerPixel(bytesPerPixel))
  {
    logger(LOG_ERROR) << "Could not get supported video modes or current bytes per pixel, to get nearest valid frame size" << std::endl;
    return false;
  }
  
  int minScore = 0;
  IndexType index = -1;
  int area = forFrameSize.height*forFrameSize.width;
  
  if(supportedVideoModes.size() > 0)
  {
    for(IndexType i = 0; i < supportedVideoModes.size(); i++)
    {
      auto &a = supportedVideoModes[i];
      
      int scoreA = a.frameSize.width*a.frameSize.height;
      scoreA = (bytesPerPixel != a.bytesPerPixel || scoreA < area || 
        a.frameSize.width < forFrameSize.width || a.frameSize.height < forFrameSize.height)?0:scoreA;
      
      if(scoreA < minScore)
      {
        minScore = scoreA;
        index = i;
      }
    }
    
    if(index < 0)
    {
      logger(LOG_ERROR) << "ToFCamera: No supported frame size exists close to the desired frame size. Could not set frame size." << std::endl;
      return false;
    }
    frameRate = supportedVideoModes[index].frameRate;
    return true;
  }
  else
  {
    logger(LOG_ERROR) << "ToFCamera: No video modes available for this depth camera." << std::endl;
    return false;
  }  
}

bool ToFCamera::_getDepthScalingFactor(float& factor)
{
  bool dealiasingEnabled;
  
  if(!get(DEALIAS_EN, dealiasingEnabled))
    return false;
  
  float illuminationFrequency;
  
  if(!_getIlluminationFrequency(illuminationFrequency))
    return false;
  
  bool frequencyCorrectionPresent = false;
  float frequencyCorrection = 1.0f;
  
  if(configFile.isPresent("calib", "freq_corr"))
  {
    frequencyCorrection = configFile.getFloat("calib", "freq_corr");
    
    if(frequencyCorrection < FLOAT_EPSILON)
      frequencyCorrectionPresent = false;
    else
      frequencyCorrectionPresent = true;
  }
  
  if(dealiasingEnabled)
  {
    bool is16BitMode;
    int dealiasedPhaseMask;
    uint ma, mb;
    
    if(!_is16BitModeEnabled(is16BitMode) || !get(DEALIASED_PHASE_MASK, dealiasedPhaseMask) 
      || !get(MA, ma) || !get(MB, mb))
      return false;
    
    if(frequencyCorrectionPresent)
      illuminationFrequency *= frequencyCorrection;
    
    if(is16BitMode)
      factor = SPEED_OF_LIGHT/1E6f/(2*(1 << 16)*illuminationFrequency)*(1 << (5 - dealiasedPhaseMask))*1.0f/ma/mb;
    else
      factor = SPEED_OF_LIGHT/1E6f/(2*(1 << 12)*illuminationFrequency)*(1 << (5 - dealiasedPhaseMask))*1.0f/ma/mb;
    
    return true;
  }
  else
  {
    if(frequencyCorrectionPresent)
      illuminationFrequency *= frequencyCorrection;
    
    factor = SPEED_OF_LIGHT/1E6f/2/illuminationFrequency/(1 << 12);
    return true;
  }
}


bool ToFCamera::_getToFFrameType(ToFFrameType &frameType) const
{
  uint r;
  
  if(!_get(ToF_FRAME_TYPE, r))
  {
    frameType = ToF_PHASE_AMPLITUDE;
    return true;
  }
  
  if(r == 1)
    frameType = ToF_I_Q;
  else if(r == 0)
    frameType = ToF_PHASE_AMPLITUDE;
  else if(r == 15)
    frameType = ToF_QUAD;
  else
    return false; // Unknown frame type
    
    return true;
}


bool ToFCamera::_reset()
{
  return set(SOFTWARE_RESET, true); // Reset the chipset
}


}
}