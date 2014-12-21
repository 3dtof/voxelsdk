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

class DepthCameraLibraryPrivate;

class DepthCameraLibrary
{
protected:
  String _libName;

  Ptr<DepthCameraLibraryPrivate> _libraryPrivate;
  
public:
  DepthCameraLibrary(const String &libName);

  bool load();
  
  inline bool isLoaded();
  
  DepthCameraFactoryPtr  getDepthCameraFactory();
  
  virtual ~DepthCameraLibrary();
};

typedef Ptr<DepthCameraLibrary> DepthCameraLibraryPtr;

}

#endif // VOXEL_DEPTHCAMERALIBRARY_H
