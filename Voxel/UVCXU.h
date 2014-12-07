/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCXU_H
#define VOXEL_UVCXU_H

#include "Device.h"

namespace Voxel
{

class UVCXU
{
protected:
  int _fd = 0;
  String _deviceNode;
  USBDevice &_usb;
  
  int _xuID;
  
  int _ioctl(int request, void *arg);
public:
  UVCXU(USBDevice &usb, int xuID);
  
  virtual ~UVCXU();
  
  bool setControl(int controlnumber, int size, uint8_t *value);
  bool getControl(int controlnumber, int size, uint8_t *value);
};

}
#endif // UVCXU_H
