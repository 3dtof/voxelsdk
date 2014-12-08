
/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCXU.h"
#include "Logger.h"
#include "USBSystem.h"

#include <string.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

namespace Voxel
{
  
UVCXU::UVCXU(USBDevice &usb, int xuID): UVC(usb), _xuID(xuID) {}

UVCXU::~UVCXU() {}

bool UVCXU::getControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;
  
  struct uvc_xu_control_query uvc;
  
  log(DEBUG) << "UVCXU: get control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_GET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    log(ERROR) << "UVCXU: " << _deviceNode << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
  return true;  
}

bool UVCXU::setControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;
  
  struct uvc_xu_control_query uvc;
  
  log(DEBUG) << "UVCXU: set control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_SET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    log(ERROR) << "UVCXU: " << _deviceNode << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
  return true;
}
  
}