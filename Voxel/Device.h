/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEVICE_H
#define VOXEL_DEVICE_H

#include "Common.h"
#include <sstream>

namespace Voxel
{
  
class VOXEL_EXPORT Device
{
public:
  enum Interface {
    USB = 0,
    LPT = 1,
    SERIAL = 2,
    I2C = 3
  };
protected:
  String _id; // in the format interface::device::serialnumber. "device" for USB devices is "vendorid:productid"
  Interface _interfaceID;
  String _deviceID;
  String _serialNumber;
  String _description;
public:
  Device(Interface interfaceid, const String &deviceid, const String &serialnumber, const String &description = ""): _interfaceID(interfaceid), 
    _deviceID(deviceid), _serialNumber(serialnumber), _description(description)
  { 
    std::ostringstream s;
    s << _interfaceID << "::" <<  _deviceID << "::" << _serialNumber;
    _id = s.str();
  }
  
  inline const String &id() const { return _id; }
  
  inline Interface interfaceID() const { return _interfaceID; }
  inline const String &deviceID() const { return _deviceID; }
  inline const String &serialNumber() const { return _serialNumber; }
  inline const String &description() const { return _description; }
  virtual ~Device() {}
};

typedef Ptr<Device> DevicePtr;

class VOXEL_EXPORT USBDevice : public Device
{
  uint16_t _vendorID, _productID; 
public:
  USBDevice(uint16_t vendorid, uint16_t productid, const String &serialnumber, const String &description = ""): 
    Device(Device::USB, getHex(vendorid) + ":" + getHex(productid), serialnumber, description), _vendorID(vendorid), _productID(productid) {}
  
  inline uint16_t vendorID() const { return _vendorID; }
  inline uint16_t productID() const { return _productID; }
  
  virtual ~USBDevice() {}
};

class VOXEL_EXPORT DeviceScanner
{
protected:
  virtual Vector<DevicePtr> _scan() = 0;
public:
  static Vector<DevicePtr> scan();
  
  virtual ~DeviceScanner() {}
};

class VOXEL_EXPORT USBDeviceScanner : public DeviceScanner
{
protected:
  virtual Vector<DevicePtr> _scan();
  friend class DeviceScanner;
  
  virtual ~USBDeviceScanner() {}
};

}

#endif // DEVICE_H
