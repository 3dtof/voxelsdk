/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCXU_H
#define VOXEL_UVCXU_H

#include "UVC.h"

namespace Voxel
{

class UVCXU: public UVC
{
protected:
  int _xuID;
public:
  UVCXU(DevicePtr usb, int xuID);
  
  virtual ~UVCXU();
  
  bool setControl(int controlnumber, int size, uint8_t *value);
  bool getControl(int controlnumber, int size, uint8_t *value);
};

typedef Ptr<UVCXU> UVCXUPtr;

}
#endif // UVCXU_H
