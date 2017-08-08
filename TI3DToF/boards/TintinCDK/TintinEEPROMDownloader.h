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
  Vector<uint8_t> _eepromData, _dataReverse;
  virtual bool _configureForDownload();
  virtual bool _download(InputFileStream &file, long unsigned int filesize);
  
  bool _printEEPROMFirst64Bytes();
  
  bool _getEEPROMStatus(uint8_t &eepromStatus);
  bool _readDataFromEEPROM(long unsigned int filesize);
  bool _checkandRewrite(long unsigned int filesize);
  bool _rewriteBlock(uint32_t startAddress, uint16_t len);
  bool _verifyBlock(uint32_t startAddress, uint32_t bytesToCheck);

public:
  TintinEEPROMDownloader(DevicePtr device): USBDownloader(device) {}
  virtual ~TintinEEPROMDownloader() {}
};
  
} 
}


#endif
