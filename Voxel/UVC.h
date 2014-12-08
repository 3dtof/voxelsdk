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

class UVC
{
protected:
  int _fd = -1;
  String _deviceNode;
  DevicePtr _usb;

public:
  UVC(DevicePtr usb);
  
  inline bool isInitialized() { return _fd >= 0; }
  
  int xioctl(int request, void *arg);
  
  virtual ~UVC();
};

}
#endif // UVCXU_H
