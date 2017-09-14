/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VoxelVader_H
#define VOXEL_TI_VoxelVader_H

#include <ToFCalculusCamera.h>
#include <Downloader.h>

#define VOXELVADER_VENDOR_ID 0x0451U
#define VOXELVADER_PRODUCT_ID 0x9108U

#define ILLUM_VOLTAGE "illum_volt" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class VoxelVader: public ToFCalculusCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  
  virtual bool _initStartParams();
  virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
  
public:
  VoxelVader(DevicePtr device);
  
  virtual ~VoxelVader() {}
};

}
}

#endif // VOXEL_TI_VoxelVader_H
