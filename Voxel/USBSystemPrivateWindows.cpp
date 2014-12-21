/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystemPrivateWindows.h"

namespace Voxel
{

bool USBSystemPrivate::isInitialized()
{
  return true;
}

  
String USBSystemPrivate::getDeviceNode(const USBDevice &usbd)
{
  return String("");
}

Vector<DevicePtr> USBSystemPrivate::getDevices()
{
  return Vector<DevicePtr>();
}

  
}