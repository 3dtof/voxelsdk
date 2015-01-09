/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystemPrivateLinux.h"

#include <libudev.h>

#include "Logger.h"

namespace Voxel
{
Vector<DevicePtr> USBSystemPrivate::getDevices()
{
  Vector<DevicePtr> devices;
  devices.reserve(10);
  
  _iterateUDevUSB([&devices](struct udev_device *dev, uint16_t vendorID, uint16_t productID, const String &serial, const String &serialIndex)
  {
    devices.push_back(DevicePtr(new USBDevice(vendorID, productID, serial, -1, "", serialIndex)));
  });
  
  return devices;
}

libusb_device *USBSystemPrivate::getDeviceHandle(const USBDevice &usbd)
{
  uint8_t busNumber, devNumber;
  
  if(!_context)
    return nullptr; // libusb is not initialized
  
  if(!getBusDevNumbers(usbd, busNumber, devNumber))
  {
    logger(LOG_ERROR) << "USBSystem: Failed to get bus and device numbers for device '" << usbd.id() << "'" << std::endl;
    return nullptr;
  }
  
  libusb_device *selected = 0;
  
  libusb_device** libusb_list;
  
  int count = libusb_get_device_list(_context, &libusb_list); 
  
  for (auto i = 0; i < count; i++)
  {
    if ((busNumber == libusb_get_bus_number(libusb_list[i])) && (devNumber == libusb_get_device_address(libusb_list[i])))
      selected = libusb_list[i];
    else
      libusb_unref_device(libusb_list[i]);
  }
  
  libusb_free_device_list(libusb_list, 0);
  
  return selected;
}

bool USBSystemPrivate::_iterateUDevUSB(Function<void(struct udev_device *dev, uint16_t vendorID, uint16_t productID, const String &serial, const String &serialIndex)> process)
{
  udev *udevHandle = udev_new();
  
  String devNode;
  
  if(!udevHandle)
  {
    logger(LOG_ERROR) << "USBSystem: UDev Init failed" << endl;
    return false;
  }
  
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices;
  struct udev_list_entry *devListEntry;
  
  /* Create a list of the devices in the 'v4l2' subsystem. */
  enumerate = udev_enumerate_new(udevHandle);
  udev_enumerate_add_match_subsystem(enumerate, "usb");
  udev_enumerate_add_match_property(enumerate, "DEVTYPE", "usb_device");
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
    struct udev_device *dev = udev_device_new_from_syspath(udevHandle, path);
    
    uint16_t vendorID, productID;
    String serial, serialIndex;
    char *endptr;
    
    vendorID = strtol(udev_device_get_sysattr_value(dev, "idVendor"), &endptr, 16);
    productID = strtol(udev_device_get_sysattr_value(dev, "idProduct"), &endptr, 16);
    
    serialIndex += udev_device_get_sysattr_value(dev, "busnum");
    serialIndex += ":";
    serialIndex += udev_device_get_sysattr_value(dev, "devpath");
    
    const char *s = udev_device_get_sysattr_value(dev, "serial");
    if(s) serial = s;
    
    process(dev, vendorID, productID, serial, serialIndex);
    
    udev_device_unref(dev);
  }
  /* Free the enumerator object */
  udev_enumerate_unref(enumerate);
  udev_unref(udevHandle);
  
  return true;
}

bool USBSystemPrivate::getBusDevNumbers(const USBDevice &usbd, uint8_t &busNumber, uint8_t &devNumber)
{
  busNumber = devNumber = 0;
  bool gotDevice = false;
  
  _iterateUDevUSB([&gotDevice, &busNumber, &devNumber, &usbd](struct udev_device *dev, uint16_t vendorID, uint16_t productID, const String &serial, const String &serialIndex)
  {
    char *endptr;
    if(!gotDevice && usbd.vendorID() == vendorID && usbd.productID() == productID &&
        (usbd.serialNumber().size() == 0 || serial == usbd.serialNumber()) &&
        (usbd.serialIndex().size() == 0 || serialIndex == usbd.serialIndex()))
      {
        busNumber = strtol(udev_device_get_sysattr_value(dev, "busnum"), &endptr, 10);
        devNumber = strtol(udev_device_get_sysattr_value(dev, "devnum"), &endptr, 10);
        gotDevice = true;
      }
  });
  
  return gotDevice;
}


// Mainly borrowed from v4l2_devices.c of guvcview
String USBSystemPrivate::getDeviceNode(const USBDevice& usbd)
{
  udev *udevHandle = udev_new();
  
  String devNode;
  
  if(!udevHandle)
  {
    logger(LOG_ERROR) << "USBSystem: Init failed to get device node" << endl;
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
    struct udev_device *dev = udev_device_new_from_syspath(udevHandle, path), *pdev, *pdev2;
    
    /* usb_device_get_devnode() returns the path to the device node
      * itself in /dev. 
      */
    const char *v4l2Device = udev_device_get_devnode(dev);
    
    /* The device pointed to by dev contains information about
      *      t he v4l2 device. In orde*r to get information about the
      *      USB device, get the parent device with the
      *      subsystem/devtype pair of "usb"/"usb_device". This will
      *      be several levels up the tree, but the function will find
      *      it.*/
    pdev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    
    if(!pdev)
    {
      logger(LOG_WARNING) << "USBSystem: Unable to find parent usb device." << endl;
      udev_device_unref(dev);
      continue;
    }
    
    if(usbd.channelID() >= 0) // Valid channel ID?
    {
      pdev2 = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_interface");
      
      if(!pdev2)
      {
        logger(LOG_WARNING) << "USBSystem: Unable to find parent usb interface." << endl;
        udev_device_unref(dev);
        continue;
      }
    }
    
    /* From here, we can call get_sysattr_value() for each file
      *      in the device's /sys entry. The strings passed into these
      *      functions (idProduct, idVendor, serial, etc.) correspond
      *      directly to the files in the directory which represents
      *      the USB device. Note that USB strings are Unicode, UCS2
      *      encoded, but the strings returned from
      *      udev_device_get_sysattr_value() are UTF-8 encoded. */
    
    uint16_t vendorID, productID;
    uint8_t channelID;
    String serial, serialIndex;
    char *endptr;
    
    vendorID = strtol(udev_device_get_sysattr_value(pdev, "idVendor"), &endptr, 16);
    productID = strtol(udev_device_get_sysattr_value(pdev, "idProduct"), &endptr, 16);
    
    if(usbd.channelID() >= 0)
      channelID = strtol(udev_device_get_sysattr_value(pdev2, "bInterfaceNumber"), &endptr, 16);
    
    if(usbd.serialIndex().size())
    {
      serialIndex += udev_device_get_sysattr_value(pdev, "busnum");
      serialIndex += ":";
      serialIndex += udev_device_get_sysattr_value(pdev, "devpath");
    }
    
    const char *s = udev_device_get_sysattr_value(pdev, "serial");
    if(s) serial = s;
    
    if(usbd.vendorID() == vendorID && usbd.productID() == productID &&
      (usbd.serialNumber().size() == 0 || serial == usbd.serialNumber()) &&
      (usbd.channelID() < 0 || channelID == usbd.channelID()) &&
      (usbd.serialIndex().size() == 0 || serialIndex == usbd.serialIndex()))
      devNode = v4l2Device;
    
    udev_device_unref(dev);
  }
  /* Free the enumerator object */
  udev_enumerate_unref(enumerate);
  
  udev_unref(udevHandle);
  
  return devNode;
}

  
}