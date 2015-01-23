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

class VOXEL_EXPORT CameraSystem
{
protected:
  Vector<DepthCameraLibraryPtr> _libraries;
  Map<String, DepthCameraFactoryPtr> _factories; // Key = device ID as returned by Device::id()
  Map<String, DepthCameraPtr> _depthCameras; // Key = device ID as returned by Device::id()
  
  Map<String, FilterFactoryPtr> _filterFactories; // Key = filter name
  
  void _init();
  
  void _loadLibraries(const Vector<String> &paths);
  
public:
  CameraSystem();
  
  bool addDepthCameraFactory(DepthCameraFactoryPtr factory);
  
  bool addFilterFactory(FilterFactoryPtr filterFactory);
  
  Vector<DevicePtr> scan();
  
  Vector<String> getSupportedFilters();
  
  DepthCameraPtr connect(const DevicePtr &device);
  
  FilterPtr createFilter(const String &name, DepthCamera::FrameType type);
  
  // Remove local reference. Outside calling function should remove reference to its DepthCamera as well
  bool disconnect(const DepthCameraPtr &depthCamera, bool reset = false);
  
  virtual ~CameraSystem();
};

}

#endif // VOXEL_CAMERASYSTEM_H
