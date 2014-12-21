/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCPRIVATE_WINDOWS_H
#define VOXEL_UVCPRIVATE_WINDOWS_H

#include <Device.h>

namespace Voxel
{
  
class UVCPrivate
{
public:
  UVCPrivate(DevicePtr usb);
  
  bool isInitialized();
  
  bool read(uint8_t *buffer, std::size_t size);
  
  virtual ~UVCPrivate();
};

}

#endif // UVCPRIVATE_H
