/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCStreamer.h"
#include "Logger.h"

#include <linux/videodev2.h>
#include <errno.h>
#include <memory.h>

namespace Voxel
{
  
UVCStreamer::UVCStreamer(Voxel::DevicePtr device): Streamer(device)
{
  _initialized = true;
  
  initForCapture();
}

bool UVCStreamer::initForCapture()
{
  if(_device->interface() != Device::USB)
    return false;
  
  _uvc = Ptr<UVC>(new UVC(_device));
  
  if(!_uvc->isInitialized())
  {
    _initialized = false;
    return false;
  }
  
  struct v4l2_capability cap;
  
  if(_uvc->xioctl(VIDIOC_QUERYCAP, &cap) == -1)
  {
    log(ERROR) << "UVCStreamer: " << _device->id() << " is not a V4L2 device" << std::endl;
    return _initialized = false;
  }
  
  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    log(ERROR) << "UVCStreamer: " << _device->id() << " is no video capture device" << std::endl;
    return _initialized = false;
  }
  
  if(!(cap.capabilities & V4L2_CAP_READWRITE))
  {
    log(ERROR) << "UVCStreamer: " << _device->id() << " does not support read()/write() calls" << std::endl;
    
    if(cap.capabilities & V4L2_CAP_STREAMING)
    {
      log(INFO) << "UVCStreamer: " << _device->id() << " supports streaming modes" << std::endl;
      
      struct v4l2_requestbuffers req;
      
      memset(&req, 0, sizeof(req));
      
      req.count = 4;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_MMAP;
      
      if(_uvc->xioctl(VIDIOC_REQBUFS, &req) == -1)
      {
        if(EINVAL == errno)
        {
          log(ERROR) << "UVCStreamer: " << _device->id() << " does not support mmap" << std::endl;
        }
      }
      else
        log(INFO) << "UVCStreamer: " << _device->id() << " supports mmap" << std::endl;
      
      memset(&req, 0, sizeof(req));
      
      req.count = 4;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_USERPTR;
      
      if(_uvc->xioctl(VIDIOC_REQBUFS, &req) == -1)
      {
        if(EINVAL == errno)
        {
          log(ERROR) << "UVCStreamer: " << _device->id() << " does not support user pointer" << std::endl;
        }
      }
      else
        log(INFO) << "UVCStreamer: " << _device->id() << " supports user pointer" << std::endl;
    }
    
    //return _initialized = false;
  }
  
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  /* Preserve original settings as set by v4l2-ctl for example */
  if(_uvc->xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    log(ERROR) << "UVCStreamer: Could not get current frame format" << std::endl;
    return _initialized = false;
  }
  
  /* Buggy driver paranoia. */
  size_t min = fmt.fmt.pix.width * 2;
  if(fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  
  if(fmt.fmt.pix.sizeimage < min)
    _frameByteSize = min;
  else
    _frameByteSize = fmt.fmt.pix.sizeimage;
  
  return _initialized = true;
}

bool UVCStreamer::capture(RawDataFramePtr p)
{
  if(!isInitialized() || !p)
    return false;
  
  if(p->data.size() != _frameByteSize)
  {
    log(ERROR) << "UVCStreamer: Frame provided is not of appropriate size." << endl;
    return false;
  }
  
  return _uvc->read(p->data.data(), _frameByteSize);
}

bool UVCStreamer::getSupportedVideoModes(Vector<VideoMode> &videoModes)
{
  if(!isInitialized())
    return false;
  
  videoModes.clear();
  
  int frameSizeIndex = 0;
  int frameRateIndex = 0;
  
  uint32_t frameWidth, frameHeight;
  
  
  struct v4l2_frmsizeenum frameSizeEnum;
  struct v4l2_frmivalenum frameIntervalEnum;
  
  while(1)
  {
    memset(&frameSizeEnum, 0, sizeof(frameSizeEnum));
    
    frameSizeEnum.index = frameSizeIndex++;
    frameSizeEnum.pixel_format = V4L2_PIX_FMT_YUYV;
    
    if(_uvc->xioctl(VIDIOC_ENUM_FRAMESIZES, &frameSizeEnum) == -1)
    {
      if(errno == EINVAL)
        break;
      else
        return false;
    }
    
    if(frameSizeEnum.type != V4L2_FRMSIZE_TYPE_DISCRETE)
    {
      log(WARNING) << "Frame types other than discrete, are not supported currently." << endl;
      continue;
    }
    
    frameWidth = frameSizeEnum.discrete.width;
    frameHeight = frameSizeEnum.discrete.height;
    
    frameRateIndex = 0;
    while(1)
    {
      memset(&frameIntervalEnum, 0, sizeof(frameIntervalEnum));
      
      frameIntervalEnum.index = frameRateIndex++;
      frameIntervalEnum.pixel_format = V4L2_PIX_FMT_YUYV;
      frameIntervalEnum.width = frameWidth;
      frameIntervalEnum.height = frameHeight;
      
      if(_uvc->xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &frameIntervalEnum) == -1)
      {
        if(errno == EINVAL)
          break;
        else
          return false;
      }
      
      if(frameIntervalEnum.type != V4L2_FRMIVAL_TYPE_DISCRETE)
      {
        log(WARNING) << "Frame interval types other than discrete, are not supported currently." << endl;
        continue;
      }
      
      VideoMode vm;
      vm.frameSize.width = frameWidth;
      vm.frameSize.height = frameHeight;
      vm.frameRate.numerator = frameIntervalEnum.discrete.denominator;
      vm.frameRate.denominator = frameIntervalEnum.discrete.numerator;
      
      videoModes.push_back(vm);
    }
  }
  return true;
}

  
}