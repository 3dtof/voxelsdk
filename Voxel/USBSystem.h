/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEM_H
#define VOXEL_USBSYSTEM_H

#include "Common.h"
#include "Device.h"

#include <libusb.h>


namespace Voxel
{
  
class USBSystemPrivate;
  
// This facilitates getting device handle corresponding a particular vid:pid:serial
class USBSystem
{
protected:
  Ptr<USBSystemPrivate> _usbPrivate;
  
public:
  USBSystem();
  
  Vector<DevicePtr> getDevices();
  
  bool isInitialized();
  
  inline USBSystemPrivate &getUSBSystemPrivate() { return *_usbPrivate; }
  
  String getDeviceNode(const USBDevice &usbd);
  
  virtual ~USBSystem() {}
};

}
#endif // VOXEL_USBSYSTEM_H
