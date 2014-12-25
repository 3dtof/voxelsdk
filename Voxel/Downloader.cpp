/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Downloader.h"
#include "USBSystem.h"
#include "Logger.h"
#include "Configuration.h"

#include <stdlib.h>

#ifdef LINUX
#include "USBSystemPrivateLinux.h"
#elif defined(WINDOWS)
#include "USBSystemPrivateWindows.h"
#include <Windows.h>
#include <winusb.h>
#define __USB200_H__ // Disable redefinition of _USB_* structs in CyAPI.h
#include "CyAPI.h"

// Borrowed from LIBUSB. See http://libusb.sourceforge.net/api-1.0/group__misc.html and http://libusb.sourceforge.net/api-1.0/group__desc.html. 
// These are used to set RequestType for WINUSB_SETUP_PACKET. See http://msdn.microsoft.com/en-us/library/windows/hardware/ff540313(v=vs.85).aspx
#define USB_RECIPIENT_DEVICE 0x00
#define USB_REQUEST_TYPE_VENDOR (0x02 << 5)
#define USB_ENDPOINT_OUT 0x00
#endif

namespace Voxel
{
  
bool Downloader::_locateFile(String &file)
{
  Configuration c;
  return c.getFirmwareFile(file);
}

#ifdef LINUX
typedef libusb_device_handle *USBHandle;
#elif defined(WINDOWS)
struct USBHandle
{
  WINUSB_INTERFACE_HANDLE winUSBHandle;
  CCyUSBDevice *device;
};
#endif

bool _configureForDownload(USBHandle device)
{
  uint8_t buffer[4];
  
  // FIXME: Explain these magic numbers
  buffer[0] = 0x89;
  buffer[1] = 0xCB;
  buffer[2] = 0x07;
  buffer[3] = 0x00;
  
#ifdef LINUX
  int status;
  status = libusb_control_transfer(device,
                                   LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                   (uint8_t) 0x05,
                                   (uint16_t) 0,
                                   (uint16_t) 0,
                                   buffer, 4, 5000);
  if (status != 4)
  {
    logger(LOG_ERROR) << "USBDownloader: Control transfer issue: Status " << status << endl;
    return false;
  }

  return true;
#elif defined(WINDOWS)

  if (device.winUSBHandle != 0)
  {
    WINUSB_SETUP_PACKET SetupPacket;
    ZeroMemory(&SetupPacket, sizeof(WINUSB_SETUP_PACKET));
    ULONG cbSent = 0;

    SetupPacket.RequestType = USB_ENDPOINT_OUT | USB_REQUEST_TYPE_VENDOR | USB_RECIPIENT_DEVICE;
    SetupPacket.Request = 0x05;
    SetupPacket.Value = 0;
    SetupPacket.Index = 0;
    SetupPacket.Length = 4;

    if (!WinUsb_ControlTransfer(device.winUSBHandle, SetupPacket, buffer, 4, &cbSent, 0))
    {
      logger(LOG_ERROR) << "USBDownloader: Control transfer issue." << std::endl;
      return false;
    }
  }
  else if (device.device)
  {
    CCyControlEndPoint *ept = device.device->ControlEndPt;
    
    ept->Target = TGT_DEVICE;
    ept->ReqType = REQ_VENDOR;
    ept->Direction = DIR_TO_DEVICE;
    ept->ReqCode = 0x05;	// SetspFileSize
    ept->Value = 0x00;
    ept->Index = 0x00;
    long length = 4;
    if (!ept->XferData(buffer, length, NULL))
    {
      logger(LOG_ERROR) << "USBDownloader: Control transfer issue." << std::endl;
    }

    return true;
  }
  else
    return false;
  
#endif
}

bool _download(USBHandle device, std::ifstream &file, long unsigned int filesize)
{
  unsigned char endpointOut = 0x06;
  unsigned char buffer[4096];
  unsigned long transferSize;
  unsigned long bytesToRead = filesize;
  unsigned long transferred = 0, t;
  int status;
  
  while (bytesToRead > 0) {
    transferSize = bytesToRead > 4096 ? 4096 : bytesToRead;
    
    file.read((char *)buffer, transferSize);
    status = file.gcount();
    
    if (status < transferSize)
      logger(LOG_DEBUG) << "USBDownloader: Read less bytes than expected" << std::endl;

#ifdef LINUX
    int rc = libusb_bulk_transfer(device, endpointOut, buffer, transferSize, &transferred, 10000);
    
    if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
    {
      logger(LOG_ERROR) << "USBDownloader: Bulk transfer failed." << endl;
      return false;
    }
#elif WINDOWS
    WINUSB_PIPE_INFORMATION  Pipe;
    ZeroMemory(&Pipe, sizeof(WINUSB_PIPE_INFORMATION));

    CCyBulkEndPoint*	dataEP;

    if (device.winUSBHandle)
    {
      if (!WinUsb_QueryPipe(device.winUSBHandle, 0, endpointOut, &Pipe))
      {
        logger(LOG_ERROR) << "USBDownloader: Could not open pipe to end point = " << endpointOut << std::endl;
        return false;
      }

      if (!WinUsb_WritePipe(device.winUSBHandle, Pipe.PipeId, buffer, transferSize, &transferred, 0))
      {
        logger(LOG_ERROR) << "USBDownloader: Could not transfer '" << transferSize << "' bytes" << std::endl;
        return false;
      }
    }
    else if (device.device)
    {
      dataEP = device.device->BulkOutEndPt;
      transferred = transferSize;
      if (!dataEP->XferData(buffer, (long &)transferred, NULL))
      {
        logger(LOG_ERROR) << "USBDownloader: Could not transfer '" << transferSize << "' bytes" << std::endl;
        return false;
      }
    }
    else
      return false;
#endif
    
    while(transferred < transferSize)
    {
#ifdef LINUX
      rc = libusb_bulk_transfer(device, endpointOut, buffer + transferred, transferSize - transferred, &t, 10000);
      
      if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
      {
        logger(LOG_ERROR) << "USBDownloader: Bulk transfer failed." << endl;
        return false;
      }
#elif defined(WINDOWS)
      if (device.winUSBHandle)
      {
        if (!WinUsb_WritePipe(device.winUSBHandle, Pipe.PipeId, buffer + transferred, transferSize - transferred, &t, 0))
        {
          logger(LOG_ERROR) << "USBDownloader: Could not transfer '" << (transferSize - transferred) << "' bytes" << std::endl;
          return false;
        }
      }
      else if (device.device)
      {
        t = transferSize - transferred;
        if (!dataEP->XferData(buffer + transferred, (long &)t, NULL))
        {
          logger(LOG_ERROR) << "USBDownloader: Could not transfer '" << t << "' bytes" << std::endl;
          return false;
        }
      }
      else
        return false;
#endif
      transferred += t;
    }
    
    bytesToRead -= transferred;
  }
  
  return true;
}


bool USBDownloader::download(const String &file)
{
  if(_device->interfaceID() != Device::USB)
  {
    logger(LOG_ERROR) << "USBDownloader: cannot download to a non-USB device" << endl;
    return false;
  }
  
  String fil = file;
  
  if(!_locateFile(fil))
  {
    logger(LOG_ERROR) << "USBDownloader: Could not locate '" << file << "'." << endl;
    return false;
  }
  
  std::ifstream f(fil, std::ios::binary | std::ios::ate);
  
  if(!f.good())
  {
    logger(LOG_ERROR) << "USBDownloader: Could not open '" << fil << "'." << endl;
    return false;
  }
  
  std::streamoff size = f.tellg();
  
  f.seekg(std::ios::beg);
  
  
  USBDevice &d = (USBDevice &)*_device;
  
  USBSystem sys;
  
  if(!sys.isInitialized())
  {
    logger(LOG_ERROR) << "USBDownloader: USBSystem init failed." << endl;
    return false;
  }
  
#ifdef LINUX
  int rc;

  libusb_device *device = sys.getUSBSystemPrivate().getDeviceHandle(d);
  
  libusb_device_handle *handle;
  
  if(device)
  {
    if((rc = libusb_open(device, &handle)) == LIBUSB_SUCCESS)
    {
      if((rc = libusb_claim_interface(handle, 0)) != LIBUSB_SUCCESS)
      {
        logger(LOG_ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
        libusb_close(handle);
        libusb_unref_device(device);
        return false;
      }
      
      bool ret = _configureForDownload(handle) && _download(handle, f, size);
      
      libusb_release_interface(handle, 0);
      libusb_close(handle);
      libusb_unref_device(device);
      
      return ret;
    }
    else
    {
      logger(LOG_ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
      libusb_unref_device(device);
      return false;
    }
  }
  else
  {
    logger(ERROR) << "USBDownloader: Failed to get device handle. Check that device is connected and is accessible from current user." << endl;
    return false;
  }
#elif defined(WINDOWS)
  String devicePath = sys.getDeviceNode(d);

  if (!devicePath.size())
  {
    logger(LOG_ERROR) << "USBDownloader: Could not get device path for '" << d.id() << "'" << std::endl;
    return false;
  }

  HANDLE handle = CreateFile(devicePath.c_str(), GENERIC_WRITE | GENERIC_READ, FILE_SHARE_WRITE | FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);

  if (handle == INVALID_HANDLE_VALUE)
  {
    logger(LOG_ERROR) << "USBDownloader: Failed to open device path '" << devicePath << "' for device '" << d.id() << "'" << std::endl;
    return false;
  }

  USBHandle usbHandle;
  usbHandle.winUSBHandle = 0;
  usbHandle.device = 0;

  if (!WinUsb_Initialize(handle, &usbHandle.winUSBHandle))
  {
    usbHandle.winUSBHandle = 0;
    logger(LOG_ERROR) << "USBDownloader: Failed to get WinUSB handle for device '" << d.id() << "'. Error: " << getDeviceError() << ". Trying CyUSB..." << std::endl;
    
    usbHandle.device = new CCyUSBDevice(handle);

    if (!usbHandle.device->IsOpen())
    {
      logger(LOG_ERROR) << "USBDownloader: Failed to get CyUSB handle for device '" << d.id() << "'." << std::endl;
      delete usbHandle.device;
      CloseHandle(handle);
      return false;
    }
  }

  bool ret = _configureForDownload(usbHandle) && _download(usbHandle, f, size);

  if (usbHandle.winUSBHandle)
    WinUsb_Free(usbHandle.winUSBHandle);

  if (usbHandle.device)
    delete usbHandle.device;

  CloseHandle(handle);

  return ret;
#endif
}
  
}