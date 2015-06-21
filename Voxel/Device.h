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
  
/**
  * \addtogroup CamSys
  * @{
  */
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
  
  String _serialIndex;
  bool _showSerialIndex;
  
  String _description;
  int _channelID; // Channel ID for multi-channel devices
  
  void _makeID()
  {
    std::ostringstream s;
    s << _interfaceID << "::" <<  _deviceID << "::" << _serialNumber;
    
    if(_showSerialIndex)
      s << ":" << _serialIndex;
      
    if(_channelID >= 0)
      s << "::" << _channelID;
      
    _id = s.str();
  }
  
public:
  Device(Interface interfaceid, const String &deviceid, const String &serialnumber, int channelID = -1, const String &description = "", const String &serialIndex = "", bool showSerialIndex = false): _interfaceID(interfaceid), 
    _deviceID(deviceid), _serialNumber(serialnumber), _description(description), _channelID(channelID), _serialIndex(serialIndex), _showSerialIndex(showSerialIndex)
  { 
    _makeID();
  }
  
  // Need to implement in all derived classes
  virtual Vector<Ptr<Device>> getDevices(const Vector<int> &channels) const { return Vector<Ptr<Device>>(); }
  
  inline const String &id() const { return _id; }
  
  inline Interface interfaceID() const { return _interfaceID; }
  inline const String &deviceID() const { return _deviceID; }
  inline const String &serialNumber() const { return _serialNumber; }
  inline void setSerialNumber(const String &s) { _serialNumber = s; }
  inline const String &serialIndex() const { return _serialIndex; }
  inline void showSerialIndex() { _showSerialIndex = true; _makeID(); }
  inline void dontShowSerialIndex() { _showSerialIndex = false; _makeID(); }
  inline const int channelID() const { return _channelID; }
  inline const String &description() const { return _description; }
  virtual ~Device() {}
};

typedef Ptr<Device> DevicePtr;

class VOXEL_EXPORT USBDevice : public Device
{
  uint16_t _vendorID, _productID; 
public:
  USBDevice(uint16_t vendorid, uint16_t productid, const String &serialnumber, int channelID = -1, const String &description = "", const String &serialIndex = "", bool showSerialIndex = false): 
    Device(Device::USB, getHex(vendorid) + ":" + getHex(productid), serialnumber, channelID, description, serialIndex, showSerialIndex), _vendorID(vendorid), _productID(productid) {}
  
  virtual Vector<DevicePtr> getDevices(const Vector<int> &channels) const;
  
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

/**
 * @}
 */

}

#endif // DEVICE_H
