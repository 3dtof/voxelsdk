/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Downloader.h"
#include "USBSystem.h"
#include "Logger.h"

#include <stdlib.h>

namespace Voxel
{
  
void Downloader::_getSearchPaths(Vector<String> &paths)
{
  paths.push_back("/lib/firmware/voxel");
  
  char *p = getenv("VOXEL_FW_PATH");
  
  if(p != 0)
  {
    String p1(p);
    
    Vector<String> splits;
    
    split(p1, ':', splits);
    
    paths.reserve(paths.size() + splits.size());
    paths.insert(paths.end(), splits.begin(), splits.end());
  }
  
  if(log.getDefaultLogLevel() >= DEBUG) 
  {
    for(auto i = 0; i < paths.size(); i++)
    {
      log(DEBUG) << paths[i];
      if(i < paths.size() - 1)
        log(DEBUG) << ":";
    }
    log(DEBUG) << endl;
  }
}

bool Downloader::_locateFile(String &file)
{
  Vector<String> paths;
  _getSearchPaths(paths);
  
  for(auto &p: paths)
  {
    std::ifstream f(p + DIR_SEP + file, std::ios::binary);
    
    if(f.good())
    {
      file = p + DIR_SEP + file;
      return true;
    }
  }
  return false;
}


bool USBDownloader::download(const String &file)
{
  if(_device->interface() != Device::USB)
  {
    log(ERROR) << "USBDownloader: cannot download to a non-USB device" << endl;
    return false;
  }
  
  String fil = file;
  
  if(!_locateFile(fil))
  {
    log(ERROR) << "USBDownloader: Could not locate '" << file << "'." << endl;
    return false;
  }
  
  std::ifstream f(fil, std::ios::binary | std::ios::ate);
  
  if(!f.good())
  {
    log(ERROR) << "USBDownloader: Could not open '" << fil << "'." << endl;
    return false;
  }
  
  long unsigned int size = f.tellg();
  
  f.seekg(std::ios::beg);
  
  USBDevice &d = (USBDevice &)*_device;
  
  USBSystem sys;
  
  int rc;
  
  if(!sys.getContext())
  {
    log(ERROR) << "USBDownloader: USBSystem init failed." << endl;
    return false;
  }
  
  libusb_device *device = sys.getDeviceHandle(d);
  
  libusb_device_handle *handle;
  
  if(device)
  {
    if((rc = libusb_open(device, &handle)) == LIBUSB_SUCCESS)
    {
      if((rc = libusb_claim_interface(handle, 0)) != LIBUSB_SUCCESS)
      {
        log(ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
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
      log(ERROR) << "USBDownloader: " << libusb_strerror((libusb_error)rc) << endl;
      libusb_unref_device(device);
      return false;
    }
  }
  else
  {
    log(ERROR) << "USBDownloader: Failed to get device handle. Check that device is connected and is accessible from current user." << endl;
    return false;
  }
  
  return true;
}

bool USBDownloader::_configureForDownload(libusb_device_handle* device)
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
    log(ERROR) << "USBDownloader: Control transfer issue: Status " << status << endl;
    return false;
  }
  
  return true;
}

bool USBDownloader::_download(libusb_device_handle *device, std::ifstream &file, long unsigned int filesize)
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
      log(DEBUG) << "USBDownloader: Read less bytes than expected" << endl;
    
    int rc = libusb_bulk_transfer(device, endpointOut, buffer, transferSize, &transferred, 10000);
    
    if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
    {
      log(ERROR) << "USBDownloader: Bulk transfer failed." << endl;
      return false;
    }
    
    while(transferred < transferSize)
    {
      rc = libusb_bulk_transfer(device, endpointOut, buffer + transferred, transferSize - transferred, &t, 10000);
      
      if(rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
      {
        log(ERROR) << "USBDownloader: Bulk transfer failed." << endl;
        return false;
      }
      transferred += t;
    }
    
    bytesToRead -= transferred;
  }
  
  return true;
}

  
}