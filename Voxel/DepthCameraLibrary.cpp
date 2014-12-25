/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DepthCameraLibrary.h"
#include "Logger.h"

#ifdef LINUX
#include <dlfcn.h>
#elif defined(WINDOWS)
#include <windows.h>
#endif

namespace Voxel
{

class DepthCameraLibraryPrivate
{
public:
#ifdef LINUX
  void *handle = 0;
#elif defined(WINDOWS)
  HINSTANCE handle;
#endif
};

std::string dynamicLoadError()
{
#ifdef WINDOWS
  DWORD error = GetLastError();
  if (error)
  {
    LPVOID lpMsgBuf;
    DWORD bufLen = FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER |
      FORMAT_MESSAGE_FROM_SYSTEM |
      FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      error,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&lpMsgBuf,
      0, NULL);
    if (bufLen)
    {
      LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
      std::string result(lpMsgStr, lpMsgStr + bufLen);

      LocalFree(lpMsgBuf);

      return result;
    }
  }
  return std::string();
#elif defined(LINUX)
  return std::string(dlerror());
#endif
}

DepthCameraLibrary::DepthCameraLibrary(const String &libName) : _libName(libName),
_libraryPrivate(Ptr<DepthCameraLibraryPrivate>(new DepthCameraLibraryPrivate())) {}
  
bool DepthCameraLibrary::load()
{
#ifdef LINUX
  _libraryPrivate->handle = dlopen(_libName.c_str(), RTLD_LAZY);
#elif defined(WINDOWS)
  _libraryPrivate->handle = LoadLibrary(_libName.c_str());
#endif

  if(!_libraryPrivate->handle) 
  {
    logger(LOG_ERROR) << "DepthCameraFactory: Failed to load " << _libName << ". Error: " << dynamicLoadError() << endl;
    return false;
  }
  return true;
}

bool DepthCameraLibrary::isLoaded() { return _libraryPrivate->handle; }
 
DepthCameraFactoryPtr DepthCameraLibrary::getDepthCameraFactory()
{
  if(!isLoaded())
    return 0;
  
  char symbol[] = "getDepthCameraFactory";

#ifdef LINUX
  GetDepthCameraFactory g = (GetDepthCameraFactory)dlsym(_libraryPrivate->handle, symbol);
#elif defined(WINDOWS)
  GetDepthCameraFactory g = (GetDepthCameraFactory)GetProcAddress(_libraryPrivate->handle, symbol);
#endif

  String error;
  if ((error = dynamicLoadError()).size())  
  {
    logger(LOG_ERROR) << "DepthCameraFactory: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << error << endl;
    return 0;
  }
  
  DepthCameraFactoryPtr p;
  (*g)(p);

  return p;
}

int DepthCameraLibrary::getABIVersion()
{
  if (!isLoaded())
    return -1;

  char symbol[] = "getABIVersion";

#ifdef LINUX
  GetABIVersion g = (GetABIVersion)dlsym(_libraryPrivate->handle, symbol);
#elif defined(WINDOWS)
  GetABIVersion g = (GetABIVersion)GetProcAddress(_libraryPrivate->handle, symbol);
#endif

  String error;
  if ((error = dynamicLoadError()).size())
  {
    logger(LOG_ERROR) << "DepthCameraFactory: Failed to load symbol " << symbol << " from library " << _libName << ". Error: " << error << endl;
    return 0;
  }

  return (*g)();
}

DepthCameraLibrary::~DepthCameraLibrary()
{
  if(isLoaded())
  {
#ifdef LINUX
    dlclose(_libraryPrivate->handle);
#elif defined(WINDOWS)
    FreeLibrary(_libraryPrivate->handle);
#endif
    _libraryPrivate->handle = 0;
  }
}


  
}