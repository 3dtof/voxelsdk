/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEMPRIVATELINUX_H
#define VOXEL_USBSYSTEMPRIVATELINUX_H

#include <libusb.h>

#include "Common.h"
#include "Device.h"

namespace Voxel
{

class USBSystemPrivate
{
protected:
  libusb_context *_context = 0;
  
public:
  USBSystemPrivate()
  {
    int ret = libusb_init(&_context);
    
    if(ret != LIBUSB_SUCCESS)
    {
      std::cerr << "Initialization of USB library failed" << std::endl;
      _context = 0;
      return;
    }
  }
  
  inline bool isInitialized() { return getContext(); }
  
  libusb_context *getContext()
  {
    return _context;
  }
  
  libusb_device *getDeviceHandle(const USBDevice &usbd);
  
  Vector<DevicePtr> getDevices();
  
  // Return /dev/videoX string corresponding to device
  String getDeviceNode(const USBDevice &usbd);
  
  virtual ~USBSystemPrivate()
  {
    if(_context)
    {
      libusb_exit(_context);
      _context = 0;
    }
  }
};


}

#endif // USBSYSTEMPRIVATELINUX_H
