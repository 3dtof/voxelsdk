/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TINTINCDKCAMERA_H
#define VOXEL_TI_TINTINCDKCAMERA_H

#include <ToFTinTinCamera.h>
#include <Downloader.h>

#include "TI3DToFExports.h"

#define TINTIN_CDK_VENDOR_ID 0x0451U
#define TINTIN_CDK_PRODUCT_ID1 0x9105U
#define TINTIN_CDK_PRODUCT_UVC 0x9106U

#define PVDD "pvdd" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage
#define TILLUM_SLAVE_ADDR "tillum_slv_addr"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT TintinCDKCamera : public ToFTinTinCamera
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
  TintinCDKCamera(DevicePtr device);
  
  virtual ~TintinCDKCamera() {}
};

}
}

#endif // VOXEL_TI_TINTINCDKCAMERA_H
