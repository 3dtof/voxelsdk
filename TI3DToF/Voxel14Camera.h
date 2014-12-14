/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXEL14CAMERA_H
#define VOXEL_TI_VOXEL14CAMERA_H

#include <ToFHaddockCamera.h>
#include <Downloader.h>

#define VOXEL_14_VENDOR_ID 0x0451
#define VOXEL_14_PRODUCT_ID1 0x9102
#define VOXEL_14_PRODUCT_ID2 0x9103

namespace Voxel
{
  
namespace TI
{

class Voxel14Camera: public ToFHaddockCamera
{
protected:
  Ptr<Downloader> _downloader;
  
  bool _init();
  
public:
  Voxel14Camera(DevicePtr device);
  
  virtual ~Voxel14Camera() {}
};

}
}

#endif // VOXEL_TI_VOXEL14CAMERA_H
