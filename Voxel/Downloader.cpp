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
#endif

namespace Voxel
{
  
bool Downloader::_locateFile(String &file)
{
  Configuration c;
  return c.getFirmwareFile(file);
}

#ifdef LINUX
bool _configureForDownload(libusb_device_handle* device)
{
  uint8_t buffer[4];
  int status;
  
  // FIXME: Explain these magic numbers
  buffer[0] = 0x89;
  buffer[1] = 0xCB;
  buffer[2] = 0x07;
  buffer[3] = 0x00;
  
  status = libusb_control_transfer(device,
                                   LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                   (uint8_t) 0x05,
                                   (uint16_t) 0,
                                   (uint16_t) 0,
                                   buffer, 4, 5000);
  if (status != 4)
  {
    logger(ERROR) << "USBDownloader: Control transfer issue: Status " << status << endl;
    return false;
  }
  
  return true;
}

bool _download(libusb_device_handle *device, std::ifstream &file, long unsigned int filesize)
{
  unsigned char endpointOut = 0x06;
  unsigned char buffer[4096];
  int transferSize;
  unsigned long bytesToRead = filesize;
  int transferred = 0, t;
  int status;
  
  while (bytesToRead > 0) {
    transferSize = bytesToRead > 4096 ? 4096 : bytesToRead;
    
    file.read((char *)buffer, transferSize);
    status = file.gcount();
    
    if (status < transferSize)
      logger(DEBUG) << "USBDownloader: Read less bytes than expected" << endl;
    
    int rc = libusb_bulk_transfer(device, endpointOut, buffer, transferSize, &transferred, 10000);
    
    if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
    {
      logger(ERROR) << "USBDownloader: Bulk transfer failed." << endl;
      return false;
    }
    
    while(transferred < transferSize)
    {
      rc = libusb_bulk_transfer(device, endpointOut, buffer + transferred, transferSize - transferred, &t, 10000);
      
      if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
      {
        logger(ERROR) << "USBDownloader: Bulk transfer failed." << endl;
        return false;
      }
      transferred += t;
    }
    
    bytesToRead -= transferred;
  }
  
  return true;
}
#endif

bool USBDownloader::download(const String &file)
{
  if(_device->interface() != Device::USB)
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
        logger(ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
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
      logger(ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
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
#endif
  
  return true;
}
  
}