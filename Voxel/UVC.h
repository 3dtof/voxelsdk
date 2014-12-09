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
  size_t size;
};

class UVC
{
protected:
  int _fd = -1;
  String _deviceNode;
  DevicePtr _usb;
  
  Vector<UVCRawData> _mappedRawData;
  
  bool _munmap(UVCRawData &data);

public:
  UVC(DevicePtr usb);
  
  inline bool isInitialized() { return _fd >= 0; }
  
  int xioctl(int request, void *arg);
  
  bool read(uint8_t *buffer, size_t size);
  
  bool mmap(uint32_t offset, UVCRawData &data);
  
  // timeout in milli-seconds
  bool isReadReady(TimeStampType timeout, bool &timedOut);
  
  virtual ~UVC();
};

}
#endif // UVCXU_H
