/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DOWNLOADER_H
#define VOXEL_DOWNLOADER_H

#include "Device.h"
#include <fstream>
#include <USBIO.h>
#include "Logger.h"

namespace Voxel
{
  
/**
 * \defgroup IO I/O classes
 * @{
 */

class VOXEL_EXPORT Downloader
{
public:
  typedef Function<void(float)> ProgressFunctionType;
  
protected:
  DevicePtr _device;
  LoggerOutStream _outStream;
  
  ProgressFunctionType _progressFunction;
  
  virtual bool _locateFile(String &file);
  
  float _progress = 0;
  inline void _setProgress(float progress /* percentage */) 
  { 
    _progress = (progress > 100?100:progress); 
    
    if(_progress < 0)
      _progress = 0;
      
    if(_progressFunction) 
      _progressFunction(progress); 
  }
  
  inline void _setProgressIncrement(float progress /* percentage */) 
  { 
    _setProgress(_progress + progress);
  }
  
public:
  Downloader(DevicePtr device): _device(device) {}
  
  virtual bool download(const String &file) = 0;
  
  inline void setLogCallback(LoggerOutStream::LoggerOutStreamFunctionType f) { _outStream.setOutputFunction(f); }
  inline void removeLogCallback() { _outStream.setOutputFunction(nullptr); }
  
  inline void setProgressFunction(ProgressFunctionType f) { _progressFunction = f; }
  inline void removeProgressFunction() { _progressFunction = nullptr; }
  inline float getProgress() { return _progress; }
  
  virtual ~Downloader() {}
};

typedef Ptr<Downloader> DownloaderPtr;

class VOXEL_EXPORT USBDownloader: public Downloader
{
protected:
  USBIOPtr _usbIO;
  
  virtual bool _configureForDownload();
  virtual bool _download(InputFileStream &file, long unsigned int filesize);
  
public:
  USBDownloader(DevicePtr device);
  
  virtual bool download(const String &file);
  
  virtual ~USBDownloader() {}
};

/**
 * @}
 */

}

#endif // VOXEL_DOWNLOADER_H
