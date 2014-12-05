/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Device.h"
#include "USBSystem.h"

namespace Voxel
{

// TODO Only USB device scanning for now. Add other interfaces later
Vector<DevicePtr> DeviceScanner::scan()
{
  USBDeviceScanner usb;
  return usb._scan();
}

Vector<DevicePtr> USBDeviceScanner::_scan()
{
  USBSystem sys;
  return sys.getDevices();
}

}