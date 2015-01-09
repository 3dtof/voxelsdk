/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXEL14CAMERA_H
#define VOXEL_TI_VOXEL14CAMERA_H

#include <ToFHaddockCamera.h>
#include <Downloader.h>

#include "TI3DToFExports.h"

#define VOXEL_14_VENDOR_ID 0x0451U
#define VOXEL_14_PRODUCT_ID1 0x9102U
#define VOXEL_14_PRODUCT_ID2 0x9103U

#define ILLUM_VOLTAGE "illum_volt" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT Voxel14Camera : public ToFHaddockCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  
  virtual bool _initStartParams();
  
public:
  Voxel14Camera(DevicePtr device);
  
  virtual ~Voxel14Camera() {}
};

}
}

#endif // VOXEL_TI_VOXEL14CAMERA_H
