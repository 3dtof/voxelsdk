/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_HADDOCKCDKCAMERA_H
#define VOXEL_TI_HADDOCKCDKCAMERA_H

#include <ToFHaddockCamera.h>
#include <Downloader.h>

#define HADDOCK_CDK_VENDOR_ID 0x0451U
#define HADDOCK_CDK_PRODUCT_ID 0x9100U

#define ILLUM_POWER_PERCENTAGE2 "illum_power_percetange2" // Illumination power percentage 2
#define ILLUM_POWER2 "illum_power2" // Illumination power 2
#define MIX_VOLTAGE "mix_volt" // Mixing voltage
#define BLK_BLANK_SIZE "blk_blank_size"

namespace Voxel
{
  
namespace TI
{

class HaddockCDKCamera: public ToFHaddockCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  virtual bool _getMaximumFrameRate ( FrameRate &frameRate, const FrameSize &forFrameSize ) const;
  
  virtual bool _initStartParams();
public:
  HaddockCDKCamera(DevicePtr device);
  
  virtual ~HaddockCDKCamera() {}
  
  friend class HaddockCDKIlluminationPowerPercentageParameter;
};

}
}

#endif // VOXEL_TI_HADDOCKCDKCAMERA_H
