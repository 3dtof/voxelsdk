/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEMPRIVATELINUX_H
#define VOXEL_USBSYSTEMPRIVATELINUX_H

#include <libusb.h>
#include <libudev.h>
#include "Common.h"
#include "Device.h"

namespace Voxel
{
  
/**
 * \addtogroup IO
 * @{
 */


class USBSystemPrivate
{
protected:
  libusb_context *_context = 0;
  
  bool _iterateUDevUSB(Function<void(struct udev_device *dev, uint16_t vendorID, uint16_t productID, const String &serial, const String &serialIndex, const String &description)> process);
  
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
  
  /// This does not look at the channel to determine which channel to use
  libusb_device *getDeviceHandle(const USBDevice &usbd);
  
  Vector<DevicePtr> getDevices();
  
  // Return /dev/videoX string corresponding to device
  String getDeviceNode(const USBDevice &usbd);
  
  bool getBusDevNumbers(const USBDevice &usbd, uint8_t &busNumber, uint8_t &devNumber);
  
  virtual ~USBSystemPrivate()
  {
    if(_context)
    {
      libusb_exit(_context);
      _context = 0;
    }
  }
};

/**
 * @}
 */

}

#endif // USBSYSTEMPRIVATELINUX_H
