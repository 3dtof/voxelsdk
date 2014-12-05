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

// This facilitates getting device handle corresponding a particular vid:pid:serial
class USBSystem
{
  libusb_context *_context = 0;
public:
  USBSystem()
  {
    int ret = libusb_init(&_context);
    
    if(ret != LIBUSB_SUCCESS)
    {
      std::cerr << "Initialization of USB library failed" << std::endl;
      _context = 0;
      return;
    }
  }
  
  Vector<DevicePtr> getDevices();
  
  libusb_context *getContext()
  {
    return _context;
  }
  
  libusb_device *getDeviceHandle(const USBDevice &usbd);
  
  // Return /dev/videoX string corresponding to device
  String getDeviceNode(const USBDevice &usbd);
  
  virtual ~USBSystem()
  {
    if(_context)
    {
      libusb_exit(_context);
      _context = 0;
    }
  }
};

}
#endif // VOXEL_USBSYSTEM_H
