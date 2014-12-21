/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystem.h"
#include "Logger.h"

#ifdef LINUX
#include "USBSystemPrivateLinux.h"
#elif defined(WINDOWS)
#include "USBSystemPrivateWindows.h"
#endif

namespace Voxel
{

USBSystem::USBSystem()
{
  _usbPrivate = Ptr<USBSystemPrivate>(new USBSystemPrivate());
}

bool USBSystem::isInitialized()
{
  return _usbPrivate->isInitialized();
}

  
Vector<DevicePtr> USBSystem::getDevices()
{
  return _usbPrivate->getDevices();
}


String USBSystem::getDeviceNode(const USBDevice& usbd)
{
  return _usbPrivate->getDeviceNode(usbd);
}


}