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
  
class TintinCDKDownloaderFactory: public DownloaderFactory
{
public:
  TintinCDKDownloaderFactory(const String &name);
  
  virtual DownloaderPtr getDownloader(DevicePtr device);
  
  virtual ~TintinCDKDownloaderFactory() {}
};
  
}
}
#endif