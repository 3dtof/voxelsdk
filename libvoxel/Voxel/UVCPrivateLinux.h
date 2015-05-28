/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCPRIVATE_LINUX_H
#define VOXEL_UVCPRIVATE_LINUX_H

#include "Common.h"
#include "Device.h"

#include "UVC.h"


namespace Voxel
{
  
/**
 * \addtogroup IO
 * @{
 */


class UVCPrivate
{
protected:
  int _fd = -1;
  String _deviceNode;
  DevicePtr _usb;
  
  Vector<UVCRawData> _mappedRawData;
  
  bool _munmap(UVCRawData &data);
  
public:
  UVCPrivate(DevicePtr usb);
  
  inline bool isInitialized() { return _fd >= 0; }
  
  int xioctl(int request, void *arg);
  
  bool read(uint8_t *buffer, std::size_t size);
  
  bool mmap(uint32_t offset, UVCRawData &data);
  
  bool clearMMap();
  
  // timeout in milli-seconds
  bool isReadReady(TimeStampType timeout, bool &timedOut);
  
  virtual ~UVCPrivate();
};

/**
 * @}
 */

}

#endif // UVCPRIVATE_H
