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
  
/**
 * \addtogroup IO
 * @{
 */


class VOXEL_EXPORT UVCRawData
{
public:
  Ptr<ByteType> data;
  std::size_t size;
};

  
class VOXEL_NO_EXPORT UVCPrivate;
  
class VOXEL_EXPORT UVC
{
protected:
  DevicePtr _usb;
  
  Ptr<UVCPrivate> _uvcPrivate;
  
public:
  UVC(DevicePtr usb);
  
  virtual bool isInitialized();
  
  inline UVCPrivate &getUVCPrivate() { return *_uvcPrivate; }
  
  bool read(uint8_t *buffer, std::size_t size);

  virtual ~UVC();
};
/**
 * @}
 */

}
#endif // UVC_H
