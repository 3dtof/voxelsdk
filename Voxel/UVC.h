/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVC_H
#define VOXEL_UVC_H

#include "Device.h"

namespace Voxel
{

class UVCRawData
{
public:
  Ptr<ByteType> data;
  std::size_t size;
};

  
class UVCPrivate;
  
class UVC
{
protected:
  DevicePtr _usb;
  
  Ptr<UVCPrivate> _uvcPrivate;
  
public:
  UVC(DevicePtr usb);
  
  bool isInitialized();
  
  inline UVCPrivate &getUVCPrivate() { return *_uvcPrivate; }
  
  bool read(uint8_t *buffer, std::size_t size);

  virtual ~UVC();
};

}
#endif // UVCXU_H
