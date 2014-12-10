/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_CAMERASYSTEM_H
#define VOXEL_CAMERASYSTEM_H

#include <DepthCameraLibrary.h>

namespace Voxel
{

class CameraSystem
{
protected:
  Vector<DepthCameraLibraryPtr> _libraries;
  Map<String, DepthCameraFactoryPtr> _factories; // Key = device ID as returned by Device::id()
  Map<String, DepthCameraPtr> _depthCameras; // Key = device ID as returned by Device::id()
  
  void _init();
  
  void _loadLibraries(const Vector<String> &paths);
  
public:
  CameraSystem();
  
  bool addDepthCameraFactory(DepthCameraFactoryPtr factory);
  
  Vector<DevicePtr> scan();
  
  DepthCameraPtr connect(DevicePtr device);
  
  virtual ~CameraSystem() {}
};

}

#endif // VOXEL_CAMERASYSTEM_H
