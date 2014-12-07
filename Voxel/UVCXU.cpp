/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCXU.h"
#include "Logger.h"
#include "USBSystem.h"

#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#define IOCTL_RETRY 4

namespace Voxel
{

int UVCXU::_ioctl(int request, void *arg)
{
  if(_fd <= 0)
    return -1;
  
  int ret = 0;
  int tries = IOCTL_RETRY;
  do
  {
    ret = ioctl(_fd, request, arg);
  }
  while (ret && tries-- && ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));
  
  if (ret && (tries <= 0)) 
    log(ERROR) << "UVCXU: ioctl (" << request << ") retried " << IOCTL_RETRY << " times - giving up: " << strerror(errno) << ")" << endl;
  
  return ret;
}

UVCXU::UVCXU(USBDevice &usb, int xuID): _usb(usb), _xuID(xuID)
{
  USBSystem sys;
  _deviceNode = sys.getDeviceNode(usb);
  
  if(_deviceNode.size() > 0)
  {
    _fd = open(_deviceNode.c_str(), O_RDWR | O_NONBLOCK);
    
    if(_fd == -1)
      log(ERROR) << "Could not open device node " << _deviceNode << ". Please check for permissions." << endl;
  }
  else
    log(ERROR) << "Could not located device node for " << _usb.id() << "." << endl;
}

UVCXU::~UVCXU()
{
  if(_fd > 0)
  {
    close(_fd);
    _fd = 0;
  }
}

bool UVCXU::getControl(int controlnumber, int size, uint8_t *value)
{
  if(_fd <= 0)
    return false;
  
  struct uvc_xu_control_query uvc;
  
  log(DEBUG) << "UVCXU: get control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_GET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (_ioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    log(ERROR) << "UVCXU: " << _deviceNode << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
  return true;  
}

bool UVCXU::setControl(int controlnumber, int size, uint8_t *value)
{
  if(_fd <= 0)
    return false;
  
  struct uvc_xu_control_query uvc;
  
  log(DEBUG) << "UVCXU: set control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << value[0] << endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_SET_CUR;
  uvc.size = size;
  uvc.data = value;
    
  if (_ioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    log(ERROR) << "UVCXU: " << _deviceNode << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
  return true;
}

}