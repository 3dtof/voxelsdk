/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_LIPSCAMERA_H
#define VOXEL_TI_LIPSCAMERA_H

#include <ToFHaddockCamera.h>
#include <Downloader.h>

#define LIPS_CAMERA_VENDOR_ID 0x0BDAU
#define LIPS_CAMERA_PRODUCT_ID1 0x5825U

#define ILLUM_VOLTAGE "illum_volt" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class LipsCamera: public ToFHaddockCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  
  virtual bool _initStartParams();
  
public:
  LipsCamera(DevicePtr device);
  
  virtual ~LipsCamera() {}
};

}
}

#endif // VOXEL_TI_LIPSCAMERA_H
