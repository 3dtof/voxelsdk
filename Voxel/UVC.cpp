/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVC.h"
#include "Logger.h"
#include "USBSystem.h"

#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#define IOCTL_RETRY 4

namespace Voxel
{
  
int UVC::xioctl(int request, void *arg)
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
    log(ERROR) << "UVC: ioctl (" << request << ") retried " << IOCTL_RETRY << " times - giving up: " << strerror(errno) << ")" << endl;
  
  return ret;
}

UVC::UVC(USBDevice &usb): _usb(usb)
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

UVC::~UVC()
{
  if(_fd > 0)
  {
    close(_fd);
    _fd = 0;
  }
}
  
}