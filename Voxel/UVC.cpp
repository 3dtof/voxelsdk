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
#include <assert.h>
#include <sys/mman.h>

#define FD_RETRY_LIMIT 4

namespace Voxel
{
  
int UVC::xioctl(int request, void *arg)
{
  if(!isInitialized())
    return -1;
  
  int ret = 0;
  int tries = FD_RETRY_LIMIT;
  do
  {
    ret = ioctl(_fd, request, arg);
  }
  while(ret && tries-- && ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));
  
  if (ret && (tries <= 0)) 
    log(ERROR) << "UVC: ioctl (" << request << ") retried " << FD_RETRY_LIMIT << " times - giving up: " << strerror(errno) << ")" << endl;
  
  return ret;
}

/// TODO This is untested code as Voxel-14 does not support read as of now
/// buffer is assumed to have capacity of atleast "size" bytes
bool UVC::read(uint8_t *buffer, size_t size)
{
  if(!isInitialized())
    return false;
  
  int64_t toRead = size;
  
  int64_t bytesRead;
  
  int retryCount = 0;
  
  while(toRead > 0)
  {
    bytesRead = ::read(_fd, (void *)(buffer + (size - toRead)), toRead);
    
    if(bytesRead > 0)
    {
      toRead -= bytesRead;
      retryCount = 0;
    }
    else if(bytesRead == -1) // error condition
    {
      if(errno == EAGAIN || errno == EWOULDBLOCK)
      {
        retryCount++;
        
        if(retryCount < FD_RETRY_LIMIT)
          continue;
        else
        {
          log(ERROR) << "UVC: Hit maximum number (" << FD_RETRY_LIMIT << ") of retries to read data" << std::endl;
          return false;
        }
      }
      else
        return false;
    }
  }
  
  assert(toRead == 0);
  
  return true;
}

bool UVC::mmap(uint32_t offset, UVCRawData &data)
{
  if(!isInitialized())
    return false;
  
  void *d = ::mmap(NULL /* start anywhere */,
                data.size,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                _fd, offset);
  
  if(d == MAP_FAILED)
    return false;
  
  data.data = Ptr<ByteType>((ByteType *)d, [](ByteType *){}); // encasing in Ptr<> with a null deleter
  
  _mappedRawData.push_back(data);
  return true;
}

bool UVC::_munmap(UVCRawData &data)
{
  if(!isInitialized())
    return false;
  
  return ::munmap((void *)&*data.data, data.size) == -1;
}

UVC::UVC(DevicePtr usb): _usb(usb)
{
  USBSystem sys;
  
  if(usb->interface() != Device::USB)
    return;
  
  _deviceNode = sys.getDeviceNode((USBDevice &)*usb);
  
  if(_deviceNode.size() > 0)
  {
    _fd = open(_deviceNode.c_str(), O_RDWR | O_NONBLOCK);
    
    if(_fd == -1)
      log(ERROR) << "Could not open device node " << _deviceNode << ". Please check for permissions." << endl;
  }
  else
    log(ERROR) << "Could not located device node for " << _usb->id() << "." << endl;
}

UVC::~UVC()
{
  if(_fd > 0)
  {
    for(auto i = 0; i < _mappedRawData.size(); i++)
      _munmap(_mappedRawData[i]);
    
    close(_fd);
    _fd = 0;
  }
}

bool UVC::isReadReady(TimeStampType timeout, bool &timedOut)
{
  timedOut = false;
  
  if(!isInitialized())
    return false;
  
  while(1)
  {
    fd_set fds;
    struct timeval tv;
    int r;
    
    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    
    /* Timeout. */
    tv.tv_sec = timeout/1000;
    tv.tv_usec = (timeout % 1000)*1000;
    
    r = select(_fd + 1, &fds, NULL, NULL, &tv);
    
    if(r == -1)
    {
      if(EINTR == errno)
        continue;
      
      return false;
    }
    
    if(r == 0)
    {
      timedOut = true;
      return false;
    }
    
    return true;
  }
}

  
}