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
  int _fd = 0;
  String _deviceNode;
  USBDevice &_usb;
  
  int _ioctl(int request, void *arg);
public:
  UVC(USBDevice &usb);
  
  inline bool isInitialized() { return _fd > 0; }
  
  virtual ~UVC();
};

}
#endif // UVCXU_H
