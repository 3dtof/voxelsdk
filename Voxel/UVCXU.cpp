
/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCXU.h"
#include "Logger.h"
#include "USBSystem.h"

#include <string.h>

#ifdef LINUX
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include "UVCPrivateLinux.h"
#elif defined(WINDOWS)
#include "UVCPrivateWindows.h"
#endif

namespace Voxel
{
  
UVCXU::UVCXU(DevicePtr usb, int xuID): UVC(usb), _xuID(xuID) {}

UVCXU::~UVCXU() {}

bool UVCXU::getControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;

#ifdef LINUX  
  struct uvc_xu_control_query uvc;
  
  memset(&uvc, 0, sizeof(uvc));
  
  logger(DEBUG) << "UVCXU: get control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << (uint)value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_GET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (getUVCPrivate().xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    logger(ERROR) << "UVCXU: " << _usb->id() << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
#elif defined(WINDOWS)
#endif
  return true;  
}

bool UVCXU::setControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;
 
#ifdef LINUX
  struct uvc_xu_control_query uvc;
  
  memset(&uvc, 0, sizeof(uvc));
  
  logger(DEBUG) << "UVCXU: set control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << (uint)value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_SET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (getUVCPrivate().xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    logger(ERROR) << "UVCXU: " << _usb->id() << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
#elif defined(WINDOWS)
#endif
  
  return true;
}
  
}