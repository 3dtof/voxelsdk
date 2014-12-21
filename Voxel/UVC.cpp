/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVC.h"
#include "Logger.h"
#include "USBSystem.h"

#include <string.h>
#include <assert.h>

#ifdef LINUX
#include "UVCPrivateLinux.h"
#elif defined(WINDOWS)
#include "UVCPrivateWindows.h"
#endif


namespace Voxel
{
  
UVC::UVC(DevicePtr usb): _usb(usb) 
{
  _uvcPrivate = Ptr<UVCPrivate>(new UVCPrivate(usb));
}
  
bool UVC::isInitialized()
{
  return _uvcPrivate->isInitialized();
}

/// TODO This is untested code as Voxel-14 does not support read as of now
/// buffer is assumed to have capacity of atleast "size" bytes
bool UVC::read(uint8_t *buffer, std::size_t size)
{
  return _uvcPrivate->read(buffer, size);
}

UVC::~UVC() {}

}