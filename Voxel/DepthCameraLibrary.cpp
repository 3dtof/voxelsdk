/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DepthCameraLibrary.h"
#include "Logger.h"

#include <dlfcn.h>

namespace Voxel
{
  
bool DepthCameraLibrary::load()
{
  _handle = dlopen(_libName.c_str(), RTLD_LAZY);
  
  if(!_handle) 
  {
    logger(ERROR) << "DepthCameraFactory: Failed to load " << _libName << ". Error: " << dlerror() << endl;
    return false;
  }
  
  return true;
}

 
DepthCameraFactoryPtr DepthCameraLibrary::getDepthCameraFactory()
{
  if(!isLoaded())
    return 0;
  
  char symbol[] = "getDepthCameraFactory";
  GetDepthCameraFactory g = (GetDepthCameraFactory)dlsym(_handle, symbol);
  
  char *error;
  if ((error = dlerror()) != NULL)  
  {
    logger(ERROR) << "DepthCameraFactory: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << dlerror() << endl;
    return 0;
  }
  
  return (*g)();
}

DepthCameraLibrary::~DepthCameraLibrary()
{
  if(_handle)
  {
    dlclose(_handle);
    _handle = 0;
  }
}


  
}