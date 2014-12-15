/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystem.h"
#include "Logger.h"

#include <libudev.h>

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
    logger(ERROR) << "USBSystem: Failed to get list of USB devices" << endl;
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
      logger(DEBUG) << "USBSystem: Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
    
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
      logger(DEBUG) << "USBSystem: Could not open device. Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
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
    logger(ERROR) << "USBSystem: Failed to get list of USB devices" << endl;
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
      logger(WARNING) << "USBSystem: Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
    
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
      logger(WARNING) << "USBSystem: Could not open device. Ignoring device " << i << ". " << libusb_strerror((libusb_error)rc) << endl;
      libusb_unref_device(device);
    }
  }
  
  libusb_free_device_list(list, 0);
  
  return selected;
}

// Mainly borrowed from v4l2_devices.c of guvcview
String USBSystem::getDeviceNode(const USBDevice& usbd)
{
  udev *udevHandle = udev_new();
  
  String devNode;
  
  if(!udevHandle)
  {
    logger(ERROR) << "USBSystem: Init failed to get device node" << endl;
    return "";
  }
  
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices;
  struct udev_list_entry *devListEntry;
  
  /* Create a list of the devices in the 'v4l2' subsystem. */
  enumerate = udev_enumerate_new(udevHandle);
  udev_enumerate_add_match_subsystem(enumerate, "video4linux");
  udev_enumerate_scan_devices(enumerate);
  devices = udev_enumerate_get_list_entry(enumerate);
  
  /*
   * For each item enumerated, print out its information.
   * udev_list_entry_foreach is a macro which expands to
   * a loop. The loop will be executed for each member in
   * devices, setting dev_list_entry to a list entry
   * which contains the device's path in /sys.
   */
  udev_list_entry_foreach(devListEntry, devices)
  {
    const char *path;
    
    /*
     * Get the filename of the /sys entry for the device
     * and create a udev_device object (dev) representing it
     */
    path = udev_list_entry_get_name(devListEntry);
    struct udev_device *dev = udev_device_new_from_syspath(udevHandle, path), *pdev;
    
    /* usb_device_get_devnode() returns the path to the device node
     * itself in /dev. 
     */
     const char *v4l2Device = udev_device_get_devnode(dev);
     
     /* The device pointed to by dev contains information about
      t he v4l2 device. In orde*r to get information about the
      USB device, get the parent device with the
      subsystem/devtype pair of "usb"/"usb_device". This will
      be several levels up the tree, but the function will find
      it.*/
     pdev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
     
     if(!pdev)
     {
       logger(WARNING) << "USBSystem: Unable to find parent usb device." << endl;
       udev_device_unref(dev);
       continue;
     }
     
     /* From here, we can call get_sysattr_value() for each file
      in the device's /sys entry. The strings passed into these
      functions (idProduct, idVendor, serial, etc.) correspond
      directly to the files in the directory which represents
      the USB device. Note that USB strings are Unicode, UCS2
      encoded, but the strings returned from
      udev_device_get_sysattr_value() are UTF-8 encoded. */
     
     uint16_t vendorID, productID;
     String serial;
     char *endptr;
     
     vendorID = strtol(udev_device_get_sysattr_value(pdev, "idVendor"), &endptr, 16);
     productID = strtol(udev_device_get_sysattr_value(pdev, "idProduct"), &endptr, 16);
     
     const char *s = udev_device_get_sysattr_value(pdev, "serial");
     if(s) serial = s;
     
     if(usbd.vendorID() == vendorID && usbd.productID() == productID &&
        (usbd.serialNumber().size() == 0 || serial == usbd.serialNumber()))
       devNode = v4l2Device;
     
     udev_device_unref(dev);
  }
  /* Free the enumerator object */
  udev_enumerate_unref(enumerate);
  
  udev_unref(udevHandle);
  
  return devNode;
}


}