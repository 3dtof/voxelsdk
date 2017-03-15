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

  
tVector<DevicePtr> USBSystem::getDevices()
{
  tVector<DevicePtr> devices = _usbPrivate->getDevices();
  
  // Show serial index for all those whose IDs have repeated.
  tMap<String, int> count;
  
  for(auto &d: devices)
  {
    auto f = count.find(d->id());
    if(f != count.end())
      f->second++;
    else
      count[d->id()] = 1;
  }
  
  for(auto &d: devices)
  {
    if(count[d->id()] > 1)
      d->showSerialIndex();
  }
  
  return devices;
}


String USBSystem::getDeviceNode(const USBDevice& usbd)
{
  return _usbPrivate->getDeviceNode(usbd);
}


}