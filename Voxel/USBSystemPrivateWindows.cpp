/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBSystemPrivateWindows.h"

#include "Logger.h"

#include <devguid.h>

namespace Voxel
{

struct DeviceInfo
{
  SP_DEVINFO_DATA devInfo;
  SP_DEVICE_INTERFACE_DATA devInterfaceData;
  Ptr<SP_DEVICE_INTERFACE_DETAIL_DATA> devInterfaceDetailData;
};

std::string getDeviceError()
{
  DWORD error = GetLastError();
  if (error)
  {
    LPVOID lpMsgBuf;
    DWORD bufLen = FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER |
      FORMAT_MESSAGE_FROM_SYSTEM |
      FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      error,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&lpMsgBuf,
      0, NULL);
    if (bufLen)
    {
      LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
      std::string result(lpMsgStr, lpMsgStr + bufLen);

      LocalFree(lpMsgBuf);

      return result;
    }
  }
  return std::string();
}

bool USBSystemPrivate::isInitialized()
{
  return true;
}

  
//String USBSystemPrivate::getDeviceNode(const USBDevice &usbd)
//{
//  return String("");
//}

bool USBSystemPrivate::_getDeviceProperty(HDEVINFO devClassInfo, DeviceInfo &devInfo, DWORD prop, String &ppBuffer)
{
  BOOL bResult;
  DWORD requiredLength = 0;
  
  ppBuffer.clear();

  bResult = SetupDiGetDeviceRegistryProperty(devClassInfo, &devInfo.devInfo, prop, NULL, NULL, 0, &requiredLength);
  
  if ((requiredLength == 0) || (bResult && GetLastError() != ERROR_INSUFFICIENT_BUFFER))
    return false;
  
  ppBuffer.resize(requiredLength - 1); // but set size to one less to not consider null character
  ppBuffer.reserve(requiredLength);// Give space for null character

  return SetupDiGetDeviceRegistryProperty(devClassInfo, &devInfo.devInfo, prop, NULL, (PBYTE)ppBuffer.data(), requiredLength, &requiredLength);
}

template <typename T>
bool USBSystemPrivate::_getDeviceProperty(HANDLE devHandle, DWORD prop, Ptr<T> &result, Function<void(T &t)> init)
{
  ULONG nBytes = 0;

  T t;

  if (init)
    init(t);

  if (!DeviceIoControl(devHandle, prop, &t, sizeof(T), &t, sizeof(T), &nBytes, NULL))
    return false;

  nBytes = t.ActualLength;

  result = byteAlloc<T>(t.ActualLength); // Actual required size with custom deleter to reconvert to byte type and delete.

  if (!result)
    return false;

  if (init)
    init(*result);

  return DeviceIoControl(devHandle, prop, &*result, nBytes, &*result, nBytes, &nBytes, NULL);
}

bool USBSystemPrivate::_iterateSetupAPI(LPGUID guid, Function<void(HDEVINFO devClassInfo, DeviceInfo &devInfo, ULONG hubIndex)> process)
{
  DWORD flags = DIGCF_PRESENT;
  
  if (guid) flags |= DIGCF_DEVICEINTERFACE;

  if (!guid) flags |= DIGCF_ALLCLASSES;

  HDEVINFO devClassInfo = SetupDiGetClassDevs(guid, NULL, NULL, flags); // | DIGCF_DEVICEINTERFACE

  if (devClassInfo == INVALID_HANDLE_VALUE)
  {
    logger(LOG_ERROR) << "USBSystem: Failed to get handle to device class. Error = " << getDeviceError() << std::endl;
    return false;
  }

  DeviceInfo deviceInfo;

  deviceInfo.devInfo.cbSize = sizeof(SP_DEVINFO_DATA);

  ULONG requiredLength;

  for (ULONG index = 0; SetupDiEnumDeviceInfo(devClassInfo, index, &deviceInfo.devInfo); index++)
  {
    deviceInfo.devInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    // FIXME: Currently this considers only the first interface of the device. Should we look for all the interfaces?
    if (!SetupDiEnumDeviceInterfaces(devClassInfo, &deviceInfo.devInfo, guid, 0, &deviceInfo.devInterfaceData))
    {
      logger(LOG_ERROR) << "USBSystem: Failed to enumerate device interfaces for index = " << index << ". Error = " << getDeviceError() << std::endl;
      SetupDiDestroyDeviceInfoList(devClassInfo);
      return false;
    }

    if (!SetupDiGetDeviceInterfaceDetail(devClassInfo, &deviceInfo.devInterfaceData, NULL, 0, &requiredLength, NULL) && GetLastError() != ERROR_INSUFFICIENT_BUFFER)
    {
      logger(LOG_ERROR) << "USBSystem: Failed to get size for interface details for index = " << index << ". Error = " << getDeviceError() << std::endl;
      SetupDiDestroyDeviceInfoList(devClassInfo);
      return false;
    }

    deviceInfo.devInterfaceDetailData = byteAlloc<SP_DEVICE_INTERFACE_DETAIL_DATA>(requiredLength);

    deviceInfo.devInterfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

    if (!SetupDiGetDeviceInterfaceDetail(devClassInfo, &deviceInfo.devInterfaceData, &*deviceInfo.devInterfaceDetailData, requiredLength, &requiredLength, NULL))
    {
      logger(LOG_ERROR) << "USBSystem: Failed to get interface details for index = " << index << ". Error = " << getDeviceError() << std::endl;
      SetupDiDestroyDeviceInfoList(devClassInfo);
      return false;
    }

    process(devClassInfo, deviceInfo, index);
  }
  SetupDiDestroyDeviceInfoList(devClassInfo);
  return true;
}

void USBSystemPrivate::_enumerateHub(const String &hubName, Function<void(HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device)> process)
{
  String deviceName = "\\\\.\\" + hubName;

  HANDLE hHubDevice = CreateFile(deviceName.c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

  if (hHubDevice == INVALID_HANDLE_VALUE)
  {
    logger(LOG_ERROR) << "USBSystem: Could not open handle to device = '" << deviceName << "'" << std::endl;
    return;
  }

  USB_NODE_INFORMATION hubInfo;

  ULONG nBytes = 0;

  if (!DeviceIoControl(hHubDevice, IOCTL_USB_GET_NODE_INFORMATION, &hubInfo, sizeof(USB_NODE_INFORMATION), &hubInfo, sizeof(USB_NODE_INFORMATION), &nBytes, NULL))
  {
    logger(LOG_ERROR) << "USBSystem: Could not get information about USB hub device = '" << deviceName << "'" << std::endl;
    CloseHandle(hHubDevice);
    return;
  }

  ULONG numPorts = hubInfo.u.HubInformation.HubDescriptor.bNumberOfPorts;

  // Support for information + upto 30 end point informations
  nBytes = sizeof(USB_NODE_CONNECTION_INFORMATION) + sizeof(USB_PIPE_INFO) * 30;

  Ptr<USB_NODE_CONNECTION_INFORMATION> connectionInfo = byteAlloc<USB_NODE_CONNECTION_INFORMATION>(nBytes);

  for (ULONG index = 1; index <= numPorts; index++)
  {
    connectionInfo->ConnectionIndex = index;

    if (!DeviceIoControl(hHubDevice, IOCTL_USB_GET_NODE_CONNECTION_INFORMATION, &*connectionInfo, nBytes, &*connectionInfo, nBytes, &nBytes, NULL))
    {
      logger(LOG_ERROR) << "USBSystem: Could not get information about device = " << index << " connected to USB hub device = '" << deviceName << "'" << std::endl;
      continue;
    }

    if (connectionInfo->ConnectionStatus != DeviceConnected)
      continue;

    if (connectionInfo->DeviceIsHub)
    {
      Ptr<USB_NODE_CONNECTION_NAME> usbNodeName;

      if (!_getDeviceProperty<USB_NODE_CONNECTION_NAME>(hHubDevice, IOCTL_USB_GET_NODE_CONNECTION_NAME, usbNodeName, [index](USB_NODE_CONNECTION_NAME &t) { t.ConnectionIndex = index; }))
      {
        logger(LOG_ERROR) << "USBSystem: Could not get node name of device = " << index << " connected to USB hub device = '" << deviceName << "'" << std::endl;
        continue;
      }

      _bstr_t b(usbNodeName->NodeName);
      String nodeName = b;

      _enumerateHub(nodeName, process);
    }
    else
    {
      Ptr<USB_NODE_CONNECTION_DRIVERKEY_NAME> usbNodeName;

      if (!_getDeviceProperty<USB_NODE_CONNECTION_DRIVERKEY_NAME>(hHubDevice, IOCTL_USB_GET_NODE_CONNECTION_DRIVERKEY_NAME, usbNodeName, [index](USB_NODE_CONNECTION_DRIVERKEY_NAME &t) { t.ConnectionIndex = index; }))
      {
        logger(LOG_ERROR) << "USBSystem: Could not get node name of device = " << index << " connected to USB hub device = '" << deviceName << "'" << std::endl;
        continue;
      }

      _bstr_t b(usbNodeName->DriverKeyName);
      String driverKeyName = b;

      String product, manufacturer, serialNumber, description;

      bool ret1 = _getStringDescriptor(hHubDevice, index, connectionInfo->DeviceDescriptor.iProduct, product);
      bool ret2 = _getStringDescriptor(hHubDevice, index, connectionInfo->DeviceDescriptor.iManufacturer, manufacturer);
      bool ret3 = _getStringDescriptor(hHubDevice, index, connectionInfo->DeviceDescriptor.iSerialNumber, serialNumber);

      if (!(ret1 && ret2 && ret3))
      {
        logger(LOG_DEBUG) << "USBSystem: Could not get request string descriptors for connection index = " << index << " connected to USB hub device = '" << deviceName << "'" << std::endl;
      }

      if (manufacturer.size() && product.size())
        description = manufacturer + " - " + product;
      else if (manufacturer.size())
        description = manufacturer;
      else if (product.size())
        description = product;
      
      if (process)
        process(hHubDevice, index, driverKeyName, DevicePtr(new USBDevice(connectionInfo->DeviceDescriptor.idVendor, connectionInfo->DeviceDescriptor.idProduct, serialNumber, description)));
    }
  }

  CloseHandle(hHubDevice);
}

bool USBSystemPrivate::_getStringDescriptor(HANDLE devHandle, ULONG index, UCHAR descriptorIndex, String &descriptor)
{
  ULONG   nBytes = sizeof(USB_DESCRIPTOR_REQUEST) + MAXIMUM_USB_STRING_LENGTH;
  ULONG   nBytesReturned = 0;

  if (descriptorIndex == 0) // No descriptor available
  {
    descriptor.clear();
    return true;
  }

  Ptr<USB_DESCRIPTOR_REQUEST> request = byteAlloc<USB_DESCRIPTOR_REQUEST>(nBytes);

  USB_STRING_DESCRIPTOR *stringDescriptor = (USB_STRING_DESCRIPTOR *)(request.get() + 1); // Appended to the end after sizeof(USB_DESCRIPTOR_REQUEST)

  // Zero fill the entire request structure
  //
  memset(request.get(), 0, nBytes);

  // Indicate the port from which the descriptor will be requested
  //
  request->ConnectionIndex = index;

  //
  // USBHUB uses URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE to process this
  // IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION request.
  //
  // USBD will automatically initialize these fields:
  //     bmRequest = 0x80
  //     bRequest  = 0x06
  //
  // We must inititialize these fields:
  //     wValue    = Descriptor Type (high) and Descriptor Index (low byte)
  //     wIndex    = Zero (or Language ID for String Descriptors)
  //     wLength   = Length of descriptor buffer
  //
  request->SetupPacket.wValue = (USB_STRING_DESCRIPTOR_TYPE << 8) | descriptorIndex;

  request->SetupPacket.wIndex = 0; // Using language ID zero

  request->SetupPacket.wLength = (USHORT)(nBytes - sizeof(USB_DESCRIPTOR_REQUEST));

  // Now issue the get descriptor request.
  //
  if (!DeviceIoControl(devHandle, IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION, &*request, nBytes, &*request, nBytes, &nBytesReturned, NULL))
  {
    logger(LOG_DEBUG) << "USBSystem: Could not get string descriptor = " << (uint)descriptorIndex << " for connection index = " << index << std::endl;
    return false;
  }

  if (nBytesReturned < 2 || stringDescriptor->bDescriptorType != USB_STRING_DESCRIPTOR_TYPE || 
    (stringDescriptor->bLength != nBytesReturned - sizeof(USB_DESCRIPTOR_REQUEST)) || 
    stringDescriptor->bLength % 2 != 0)
  {
    logger(LOG_DEBUG) << "USBSystem: Got invalid string descriptor = " << (uint)descriptorIndex << " for connection index = " << index << std::endl;
    return false;
  }

  _bstr_t b(stringDescriptor->bString);
  descriptor = b;
  
  return true;
}

bool USBSystemPrivate::_iterateOverAllDevices(LPGUID guid, Function<void(HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device)> process)
{
  return _iterateSetupAPI(guid, [this, &process](HDEVINFO devClassInfo, DeviceInfo &devInfo, ULONG hubIndex) {
    HANDLE hHCDev = CreateFile(devInfo.devInterfaceDetailData->DevicePath, GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

    if (hHCDev != INVALID_HANDLE_VALUE)
    {
      Ptr<USB_ROOT_HUB_NAME> rootHubName;
      if (!_getDeviceProperty(hHCDev, IOCTL_USB_GET_ROOT_HUB_NAME, rootHubName))
        return;

      _bstr_t b(rootHubName->RootHubName);
      
      String hubName = b;

      _enumerateHub(hubName, process);
    }

    CloseHandle(hHCDev);
  });
}

Vector<DevicePtr> USBSystemPrivate::getDevices()
{
  Vector<DevicePtr> devices;
  devices.reserve(10); // Reserve some 10 slots

  bool ret = _iterateOverAllDevices((LPGUID)&GUID_CLASS_USB_HOST_CONTROLLER, [&devices](HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device) {
    devices.push_back(device);
  });

  return devices;
}

String USBSystemPrivate::getDeviceNode(const USBDevice &usbd)
{
  Map<String, DeviceInfo> deviceMap; // driver key name -> DeviceInfo map

  bool ret = _iterateSetupAPI((LPGUID)&GUID_DEVINTERFACE_USB_DEVICE, [this, &deviceMap](HDEVINFO devClassInfo, DeviceInfo &devInfo, ULONG hubIndex) {
    String driverKeyName;
    if (!_getDeviceProperty(devClassInfo, devInfo, SPDRP_DRIVER, driverKeyName))
    {
      logger(LOG_ERROR) << "USBSystem: Failed to driver key name for index = " << hubIndex << ". Error = " << getDeviceError() << std::endl;
      return;
    }

    if (!driverKeyName.size())
    {
      logger(LOG_ERROR) << "USBSystem: Got blank driver key name for index = " << hubIndex << ". Error = " << getDeviceError() << std::endl;
      return;
    }

    deviceMap[driverKeyName] = devInfo;
  });

  
  if (!ret)
    return "";

  String deviceNodePath;

  bool pathSet = false;
  ret = _iterateOverAllDevices((LPGUID)&GUID_CLASS_USB_HOST_CONTROLLER, [&usbd, &deviceNodePath, &pathSet, &deviceMap](HANDLE hubDevice, ULONG portIndex, const String &driverKeyName, DevicePtr &device) {

    if (pathSet)
      return;

    if (device->interfaceID() != Device::USB)
      return;

    USBDevice &u = (USBDevice &)*device;

    if (usbd.vendorID() == u.vendorID() && usbd.productID() == u.productID() &&
      (!usbd.serialNumber().size() || usbd.serialNumber() == u.serialNumber()))
    {
      auto d = deviceMap.find(driverKeyName);

      if (d == deviceMap.end())
      {
        logger(LOG_WARNING) << "USBSystem: Got null entry in driver key name = " << driverKeyName << ". Ignoring this device" << std::endl;
        return;
      }

      deviceNodePath = d->second.devInterfaceDetailData->DevicePath;
      pathSet = true;
    }
  });

  return deviceNodePath;
}

bool USBSystemPrivate::getDevInst(const String &devicePath, DWORD &devInst, LPGUID guid)
{
  bool devInstSet = false;
  bool ret = _iterateSetupAPI(guid, [this, &devicePath, &devInst, &devInstSet](HDEVINFO devClassInfo, DeviceInfo &devInfo, ULONG hubIndex) {
    
    _bstr_t b(devInfo.devInterfaceDetailData->DevicePath);

    String devPath = b;

    if (!devInstSet && devPath == devicePath)
    {
      devInst = devInfo.devInfo.DevInst;
      devInstSet = true;
    }
  });

  return devInstSet;
}

}