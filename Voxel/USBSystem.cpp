/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystem.h"
#include "Logger.h"

namespace Voxel
{
  
Vector<DevicePtr> USBSystem::getDevices()
{
  libusb_device **list = 0;
  int rc = 0;
  int count = 0;
  
  Vector<DevicePtr> devices;
  
  if(!_context)
  {
    return devices; // libusb is not initialized
  }
  
  count = libusb_get_device_list(_context, &list);
  
  if(count <= 0)
  {
    log(ERROR) << "USBSystem: Failed to get list of USB devices" << endl;
    return devices;
  }
  
  devices.reserve(count);
  
  unsigned char buf[64];
  
  for (auto i = 0; i < count; i++) {
    libusb_device *device = list[i];
    libusb_device_descriptor desc = {0};
    libusb_device_handle *deviceHandle;
    
    rc = libusb_get_device_descriptor(device, &desc);
    
    if(rc != LIBUSB_SUCCESS)
      log(WARNING) << "USBSystem: Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
    
    if ((rc = libusb_open(device, &deviceHandle)) == 0)
    {
      int bytes = libusb_get_string_descriptor_ascii(deviceHandle, desc.iSerialNumber, buf, sizeof(buf));
      
      if(bytes > 0)
      {
        devices.push_back(DevicePtr(new USBDevice(desc.idVendor, desc.idProduct, String((char *)buf, bytes))));
      }
      else
      {
        devices.push_back(DevicePtr(new USBDevice(desc.idVendor, desc.idProduct, "")));
        //std::cerr << "Device " << getHex(desc.idVendor) << ":" << getHex(desc.idProduct) << "  has no serial number" << endl;
      }
      
      libusb_close(deviceHandle);
    }
    else
      log(WARNING) << "USBSystem: Could not open device. Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
  }
  
  libusb_free_device_list(list, 1);
  
  return devices;
}

libusb_device *USBSystem::getDeviceHandle(const USBDevice &usbd)
{
  libusb_device *selected = 0;
  
  libusb_device **list = 0;
  int rc = 0;
  int count = 0;
  
  Vector<DevicePtr> devices;
  
  if(!_context)
  {
    return selected; // libusb is not initialized
  }
  
  count = libusb_get_device_list(_context, &list);
  
  if(count <= 0)
  {
    log(ERROR) << "USBSystem: Failed to get list of USB devices" << endl;
    return selected;
  }
  
  unsigned char buf[64];
  
  String serialNumber;
  
  for (auto i = 0; i < count; i++) {
    libusb_device *device = list[i];
    libusb_device_descriptor desc = {0};
    libusb_device_handle *deviceHandle;
    
    rc = libusb_get_device_descriptor(device, &desc);
    
    if(rc != LIBUSB_SUCCESS)
      log(WARNING) << "USBSystem: Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
    
    if ((rc = libusb_open(device, &deviceHandle)) == 0)
    {
      int bytes = libusb_get_string_descriptor_ascii(deviceHandle, desc.iSerialNumber, buf, sizeof(buf));
      
      if(bytes > 0)
        serialNumber = String((char *)buf, bytes);
      else
        serialNumber = "";
      
      libusb_close(deviceHandle);
      
      if(!selected && usbd.productID() == desc.idProduct && usbd.vendorID() == desc.idVendor 
          && (usbd.serialNumber().size() == 0 || usbd.serialNumber() == serialNumber))
        selected = device;
      else
        libusb_unref_device(device);
    }
    else
    {
      log(WARNING) << "USBSystem: Could not open device. Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
      libusb_unref_device(device);
    }
  }
  
  libusb_free_device_list(list, 0);
  
  return selected;
}

  
}