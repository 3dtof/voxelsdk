/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TINTIN_EEPROM_DOWNLOADER_H
#define VOXEL_TINTIN_EEPROM_DOWNLOADER_H

#include "Downloader.h"

namespace Voxel
{
  
namespace TI
{
  
class TintinEEPROMDownloader: public USBDownloader
{
protected:
  virtual bool _configureForDownload();
  virtual bool _download(InputFileStream &file, long unsigned int filesize);
  
  bool _printEEPROMFirst64Bytes();
  
  bool _getEEPROMStatus(uint8_t &eepromStatus);
  
public:
  TintinEEPROMDownloader(DevicePtr device): USBDownloader(device) {}
  virtual ~TintinEEPROMDownloader() {}
};
  
} 
}


#endif
