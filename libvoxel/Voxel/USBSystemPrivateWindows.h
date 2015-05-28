/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBSYSTEMPRIVATEWINDOWS_H
#define VOXEL_USBSYSTEMPRIVATEWINDOWS_H

#include <Device.h>

#define INITGUID
#include <Windows.h>
#include <SetupAPI.h>
#include <usbiodef.h>
#include <comdef.h>
#include <usbioctl.h>

namespace Voxel
{
  
/**
 * \addtogroup IO
 * @{
 */


struct DeviceInfo;
typedef void *HANDLE;
typedef void *HDEVINFO;
typedef unsigned long DWORD;
typedef unsigned long ULONG;
typedef unsigned char UCHAR;

std::string getDeviceError();

// Code here is mostly borrowed from
// template code of Visual C++ application - http://code.msdn.microsoft.com/USBView-sample-application-e3241039
class VOXEL_NO_EXPORT USBSystemPrivate
{
  bool _getDeviceProperty(HDEVINFO devClassInfo, DeviceInfo &devInfo, DWORD prop, String &result);
  
  template <typename T>
  bool _getDeviceProperty(HANDLE devHandle, DWORD prop, Ptr<T> &result, Function<void(T &t)> init = nullptr);

  bool _iterateSetupAPI(LPGUID guid, Function<void(HDEVINFO devClassInfo, DeviceInfo &devInfo, ULONG hubIndex)> process);

  bool _iterateOverAllDevices(LPGUID guid, Function<void(HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device)> process);

  void _enumerateHub(int busIndex, const String &hubIndex, const String &hubName, Function<void(HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device)> process);

  bool _getStringDescriptor(HANDLE devHandle, ULONG index, UCHAR descriptorIndex, String &descriptor);

public:
  USBSystemPrivate() {}
  
  bool isInitialized();
  
  Vector<DevicePtr> getDevices();
  
  String getDeviceNode(const USBDevice &usbd);

  bool getDevInst(const String &devicePath, DWORD &devInst, LPGUID guid = (LPGUID)&GUID_CLASS_USB_DEVICE);
  
  virtual ~USBSystemPrivate() {}
};
/**
 * @}
 */

}

#endif // USBSYSTEMPRIVATE_H
