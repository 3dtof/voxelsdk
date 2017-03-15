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
tVector<DevicePtr> DeviceScanner::scan()
{
  USBDeviceScanner usb;
  return usb._scan();
}

tVector<DevicePtr> USBDeviceScanner::_scan()
{
  USBSystem sys;
  return sys.getDevices();
}

tVector<DevicePtr> USBDevice::getDevices(const tVector<int> &channels) const
{
  tVector<DevicePtr> devices;
  for(auto i = 0; i < channels.size(); i++)
  {
    devices.push_back(DevicePtr(new USBDevice(vendorID(), productID(), serialNumber(), channels[i], description(), serialIndex(), _showSerialIndex)));
  }
  return devices;
}


}