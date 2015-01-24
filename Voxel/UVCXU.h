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
/**
 * \addtogroup IO
 * @{
 */

  
class VOXEL_EXPORT UVCXU : public UVC
{
protected:
  int _xuID;

  class VOXEL_NO_EXPORT UVCXUPrivate;
  Ptr<UVCXUPrivate> _uvcXUPrivate;

public:
  UVCXU(DevicePtr usb, int xuID);

  bool isInitialized();
  
  virtual ~UVCXU();
  
  bool setControl(int controlnumber, int size, uint8_t *value);
  bool getControl(int controlnumber, int size, uint8_t *value);
};

typedef Ptr<UVCXU> UVCXUPtr;
/**
 * @}
 */

}
#endif // UVCXU_H
