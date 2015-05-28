/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DOWNLOADER_FACTORY_H
#define VOXEL_DOWNLOADER_FACTORY_H

#include "Downloader.h"
#include "Common.h"
#include "Device.h"

namespace Voxel
{
  
/**
 *  \addtogroup CamSys
 *  @{
 */

class VOXEL_EXPORT DownloaderFactory
{
protected:
  Vector<DevicePtr> _supportedDevices;
  
  inline void _addSupportedDevices(const Vector<DevicePtr> &devices);
  
  String _name;
  
public:
  DownloaderFactory(const String &name): _name(name) {}
  
  inline const String &name() const { return _name; }
  
  inline const Vector<DevicePtr> &getSupportedDevices() const { return _supportedDevices; }
  
  // Instantiate a depth camera for the specified device
  virtual DownloaderPtr getDownloader(DevicePtr device) = 0;
  
  virtual ~DownloaderFactory() {}
};

void DownloaderFactory::_addSupportedDevices(const Vector<DevicePtr> &devices)
{
  _supportedDevices.reserve(_supportedDevices.size() + devices.size());
  _supportedDevices.insert(_supportedDevices.end(), devices.begin(), devices.end());
}


// Implement this function in every device-specific voxel library

typedef Ptr<DownloaderFactory> DownloaderFactoryPtr;

#ifndef SWIG
extern "C" void getDownloaderFactory(DownloaderFactoryPtr &downloaderFactory);
#endif

typedef void (*GetDownloaderFactory)(DownloaderFactoryPtr &downloaderFactory); // Function type to return DownloaderFactory

/**
 * @}
 */
}

#endif // DEPTHCAMERA_FACTORY_H
