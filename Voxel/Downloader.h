/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DOWNLOADER_H
#define VOXEL_DOWNLOADER_H

#include "Device.h"

#include <libusb.h>
#include <fstream>

namespace Voxel
{

class Downloader
{
protected:
  DevicePtr _device;
  
  virtual bool _locateFile(String &file);
  
public:
  Downloader(DevicePtr device): _device(device) {}
  
  virtual bool download(const String &file) = 0;
  
  virtual ~Downloader() {}
};

class USBDownloader: public Downloader
{
public:
  USBDownloader(DevicePtr device): Downloader(device) {}
  
  virtual bool download (const String &file);
  
  virtual ~USBDownloader() {}
};

}

#endif // VOXEL_DOWNLOADER_H
