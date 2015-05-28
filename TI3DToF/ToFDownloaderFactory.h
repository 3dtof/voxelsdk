/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_TOF_DOWNLOADER_FACTORY_H
#define VOXEL_TI_TOF_DOWNLOADER_FACTORY_H

#include "DownloaderFactory.h"

namespace Voxel
{
  
namespace TI
{
  
class ToFDownloaderFactory: public DownloaderFactory
{
public:
  ToFDownloaderFactory(const String &name);
  
  virtual DownloaderPtr getDownloader(DevicePtr device);
  
  virtual ~ToFDownloaderFactory() {}
};
  
}
}
#endif