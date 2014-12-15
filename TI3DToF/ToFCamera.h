/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOFCAMERA_H
#define VOXEL_TI_TOFCAMERA_H

#include <DepthCamera.h>
#include <RegisterProgrammer.h>
#include <Streamer.h>

// These parameters are to be defined in the derived classes using the following names
#define INTG_TIME "intg_time"  // Integration time
#define ILLUM_POWER "illum_power" // Illumination power
#define ILLUM_FREQ1 "illum_freq1" // Illumination frequency (MHz)
#define ILLUM_FREQ2 "illum_freq2" // Illumination frequency (MHz)

namespace Voxel
{
  
namespace TI
{

class ToFCamera: public DepthCamera
{
protected:
  Ptr<RegisterProgrammer> _programmer;
  Ptr<Streamer> _streamer;
  
  RawDataFramePtr _rawDataFrame; // Used by _captureDepthFrame(). This is not exposed to DepthCamera
  virtual bool _captureRawUnprocessedFrame(RawFramePtr &rawFrame);
  virtual bool _convertToDepthFrame(RawFramePtr &rawFrame, DepthFramePtr &depthFrame);
  
  virtual bool _start();
  virtual bool _stop();
  
  virtual bool _initStartParams() = 0;
  
public:
  ToFCamera(const String &name, DevicePtr device): DepthCamera(name, device) {}
  
  virtual bool isInitialized()
  {
    return _programmer and _programmer->isInitialized() and 
            _streamer and _streamer->isInitialized();
  }
  
  virtual ~ToFCamera() {}
};

}
}

#endif // TOFCAMERA_H
