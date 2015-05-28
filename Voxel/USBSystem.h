/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEM_H
#define VOXEL_USBSYSTEM_H

#include "Common.h"
#include "Device.h"

namespace Voxel
{
  
/**
 * \addtogroup IO
 * @{
 */

  
class VOXEL_NO_EXPORT USBSystemPrivate;
  
// This facilitates getting device handle corresponding a particular vid:pid:serial
class VOXEL_EXPORT USBSystem
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
/**
 * @}
 */

}
#endif // VOXEL_USBSYSTEM_H
