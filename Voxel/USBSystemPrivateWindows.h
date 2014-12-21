/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEMPRIVATEWINDOWS_H
#define VOXEL_USBSYSTEMPRIVATEWINDOWS_H

#include <Device.h>

namespace Voxel
{
  
class USBSystemPrivate
{
public:
  USBSystemPrivate() {}
  
  bool isInitialized();
  
  Vector<DevicePtr> getDevices();
  
  String getDeviceNode(const USBDevice &usbd);
  
  virtual ~USBSystemPrivate() {}
};

}

#endif // USBSYSTEMPRIVATE_H
