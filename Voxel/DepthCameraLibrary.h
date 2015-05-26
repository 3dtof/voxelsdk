/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEPTHCAMERALIBRARY_H
#define VOXEL_DEPTHCAMERALIBRARY_H

#include <Common.h>
#include <DepthCameraFactory.h>
#include <Filter/FilterFactory.h>
#include "DownloaderFactory.h"

namespace Voxel
{

/**
  * \addtogroup CamSys
  * @{
  */

class VOXEL_NO_EXPORT DepthCameraLibraryPrivate;

class VOXEL_EXPORT DepthCameraLibrary
{
protected:
  String _libName;

  Ptr<DepthCameraLibraryPrivate> _libraryPrivate;
  
public:
  DepthCameraLibrary(const String &libName);

  bool load();
  
  inline bool isLoaded();
  
  DepthCameraFactoryPtr getDepthCameraFactory();
  
  FilterFactoryPtr getFilterFactory();
  
  DownloaderFactoryPtr getDownloaderFactory();

  int getABIVersion(); // Get DepthCameraLibrary's ABI version
  
  virtual ~DepthCameraLibrary();
};

typedef Ptr<DepthCameraLibrary> DepthCameraLibraryPtr;

/**
 * @}
 */
}

#endif // VOXEL_DEPTHCAMERALIBRARY_H
