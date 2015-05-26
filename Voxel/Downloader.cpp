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

namespace Voxel
{
  
bool Downloader::_locateFile(String &file)
{
  Configuration c;
  
  if(c.getFirmwareFile(file))
    return true;
  
  // Try name as is, to see whether it is valid by itself. That is, 
  // it could be an absolute path or path relative to current working directory
  std::ifstream f(file, std::ios::binary);
  
  if(f.good())
    return true;
  return false;
}

bool USBDownloader::_configureForDownload()
{
  if(!_usbIO->isInitialized())
  {
    logger(LOG_ERROR) << "USBDownloader: USBIO not initialized" << std::endl;
    _outStream << "USBDownloader: USBIO not initialized" << std::endl;
    return false;
  }
  
  uint8_t buffer[4];
  
  // FIXME: Explain these magic numbers
  buffer[0] = 0x89;
  buffer[1] = 0xCB;
  buffer[2] = 0x07;
  buffer[3] = 0x00;
  
  bool ret = _usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 0x05, 0, 0, buffer, 4, 5000);
  _setProgress(10);
  return ret;
}

bool USBDownloader::_download(InputFileStream &file, long unsigned int filesize)
{
  unsigned char endpointOut = 0x06;
  unsigned char buffer[4096];
  
  typedef long TransferSizeType;
  
  TransferSizeType transferSize;
  TransferSizeType bytesToRead = filesize;
  TransferSizeType transferred = 0, t;
  int status;
  
  float p = getProgress();
  
  while (bytesToRead > 0) {
    transferSize = bytesToRead > 4096 ? 4096 : bytesToRead;
    
    file.read((char *)buffer, transferSize);
    status = file.gcount();
    
    if (status < transferSize)
      logger(LOG_DEBUG) << "USBDownloader: Read less bytes than expected" << std::endl;
    
    if(!_usbIO->bulkTransfer(endpointOut, buffer, transferSize, transferred))
    {
      logger(LOG_ERROR) << "USBDownloader: Bulk transfer failed." << std::endl;
      return false;
    }
    
    while(transferred < transferSize)
    {
      if(!_usbIO->bulkTransfer(endpointOut, buffer + transferred, transferSize - transferred, t))
      {
        logger(LOG_ERROR) << "USBDownloader: Bulk transfer failed." << std::endl;
        return false;
      }
      transferred += t;
    }
    
    bytesToRead -= transferred;
    
    _setProgress(p + (100 - p)*(1 - bytesToRead*1.0/filesize));
  }
  
  return true;
}

USBDownloader::USBDownloader(DevicePtr device): Downloader(device), _usbIO(new USBIO(device))
{
}


bool USBDownloader::download(const String &file)
{
  _setProgress(0);
  
  String fil = file;
  
  if(!_locateFile(fil))
  {
    logger(LOG_ERROR) << "USBDownloader: Could not locate '" << file << "'." << std::endl;
    _outStream << "USBDownloader: Could not locate '" << file << "'." << std::endl;
    return false;
  }
  
  std::ifstream f(fil, std::ios::binary | std::ios::ate);
  
  if(!f.good())
  {
    logger(LOG_ERROR) << "USBDownloader: Could not open '" << fil << "'." << std::endl;
    _outStream << "USBDownloader: Could not open '" << fil << "'." << std::endl;
    return false;
  }
  
  std::streamoff size = f.tellg();
  
  f.seekg(std::ios::beg);
  
  bool ret = _configureForDownload() && _download(f, size);
  
  _setProgress(100);
  
  return ret;
}
  
}