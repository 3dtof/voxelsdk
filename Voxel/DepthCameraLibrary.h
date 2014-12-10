/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEPTHCAMERALIBRARY_H
#define VOXEL_DEPTHCAMERALIBRARY_H

#include <Common.h>
#include <DepthCameraFactory.h>

namespace Voxel
{
  
class DepthCameraLibrary
{
protected:
  String _libName;
  
  void *_handle = 0;
  
public:
  DepthCameraLibrary(String libName) { _libName = libName; }
  
  bool load();
  
  inline bool isLoaded() { return _handle; }
  
  DepthCameraFactoryPtr  getDepthCameraFactory();
  
  virtual ~DepthCameraLibrary();
};

typedef Ptr<DepthCameraLibrary> DepthCameraLibraryPtr;

}

#endif // VOXEL_DEPTHCAMERALIBRARY_H
