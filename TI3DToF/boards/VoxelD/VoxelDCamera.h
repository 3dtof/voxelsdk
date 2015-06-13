/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXELDCAMERA_H
#define VOXEL_TI_VOXELDCAMERA_H

#include <ToFTintinCamera.h>
#include <Downloader.h>

#define VOXEL_D_VENDOR_ID 0x0451U
#define VOXEL_D_PRODUCT_ID1 0x9101U
#define VOXEL_D_PRODUCT_ID2 0x9104U

#undef ILLUM_VOLTAGE
#define ILLUM_VOLTAGE "illum_volt" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class VoxelDCamera : public ToFTintinCamera
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
  VoxelDCamera(DevicePtr device);
  
  virtual ~VoxelDCamera() {}
};

}
}

#endif // VOXEL_TI_VOXEL14CAMERA_H
