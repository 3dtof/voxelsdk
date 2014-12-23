/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCStreamer.h"
#include "Logger.h"


#include <errno.h>
#include <memory.h>
#include <assert.h>

#ifdef LINUX
#include <linux/videodev2.h>
#include "UVCPrivateLinux.h"
#elif defined(WINDOWS)
#include "UVCPrivateWindows.h"
#endif

namespace Voxel
{
  
UVCStreamer::UVCStreamer(Voxel::DevicePtr device): Streamer(device)
{
  _initialized = true;
  
  if(!_uvcInit())
    _initialized = false;
}

bool UVCStreamer::_uvcInit()
{
  if(_device->interfaceID() != Device::USB)
    return false;
  
  _uvc = Ptr<UVC>(new UVC(_device));
  
  return _uvc->isInitialized();
}

bool UVCStreamer::getCurrentVideoMode(VideoMode &videoMode)
{
  if(!isInitialized())
    return false;
  
#ifdef LINUX
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not get current frame format" << std::endl;
    return false;
  }
  
  videoMode.frameSize.width = fmt.fmt.pix.width;
  videoMode.frameSize.height = fmt.fmt.pix.height;
  
  _updateFrameByteSize(fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_G_PARM, &parm) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not get current capture parameters" << std::endl;
    return false;
  }
  
  videoMode.frameRate.numerator = parm.parm.capture.timeperframe.denominator;
  videoMode.frameRate.denominator = parm.parm.capture.timeperframe.numerator;
#elif defined(WINDOWS)
#endif
  return true;
}

bool UVCStreamer::setVideoMode(const VideoMode &videoMode)
{
  if(!isInitialized() || isRunning())
    return false;
  
#ifdef LINUX
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not get current frame format" << std::endl;
    return false;
  }
  
  fmt.fmt.pix.width = videoMode.frameSize.width;
  fmt.fmt.pix.height = videoMode.frameSize.height;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_S_FMT, &fmt) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not set current frame format" << std::endl;
    return false;
  }
  
  /// Get once more to set frame size
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not set current frame format" << std::endl;
    return false;
  }
  
  _updateFrameByteSize(fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_G_PARM, &parm) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not get current capture parameters" << std::endl;
    return false;
  }
  
  parm.parm.capture.timeperframe.denominator = videoMode.frameRate.numerator;
  parm.parm.capture.timeperframe.numerator = videoMode.frameRate.denominator;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_S_PARM, &parm) == -1)
  {
    logger(ERROR) << "UVCStreamer: Could not set current capture parameters" << std::endl;
    return false;
  }
#elif defined(WINDOWS)
#endif
  return true;
}


bool UVCStreamer::_initForCapture()
{
  if(!_uvcInit())
    return false;
  
  ///// 1. Figure out which mode of capture to use
#ifdef LINUX
  struct v4l2_capability cap;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_QUERYCAP, &cap) == -1)
  {
    logger(ERROR) << "UVCStreamer: " << _device->id() << " is not a V4L2 device" << std::endl;
    return _initialized = false;
  }
  
  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    logger(ERROR) << "UVCStreamer: " << _device->id() << " is no video capture device" << std::endl;
    return _initialized = false;
  }
  
  if(cap.capabilities & V4L2_CAP_READWRITE)
  {
    logger(INFO) << "UVCStreamer: " << _device->id() << " does not support read()/write() calls" << std::endl;
    _captureMode = CAPTURE_READ_WRITE;
  }
  else if(cap.capabilities & V4L2_CAP_STREAMING)
  {
    logger(INFO) << "UVCStreamer: " << _device->id() << " supports streaming modes" << std::endl;
    _captureMode = CAPTURE_STREAMING;
  }
#elif defined(WINDOWS)
#endif
  
  //// 2. Figure out frame size and frame rate
  VideoMode currentVideoMode;
  
  if(!getCurrentVideoMode(currentVideoMode))
  {
    logger(LOG_ERROR) << "Could not get the current video mode" << std::endl;
    return _initialized = false;
  }
  
  return _initialized = true;
}

bool UVCStreamer::_start()
{
  if(!_initForCapture())
    return false;
  
#ifdef LINUX
  if(_captureMode == CAPTURE_STREAMING)
  {
    struct v4l2_requestbuffers req;
    
    memset(&req, 0, sizeof(req));
    
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if(_uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
    {
      if(EINVAL == errno)
      {
        logger(ERROR) << "UVCStreamer: " << _device->id() << " does not support mmap" << std::endl;
        
        memset(&req, 0, sizeof(req));
        
        req.count = 2;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;
        
        if(_uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
        {
          if(EINVAL == errno)
          {
            logger(ERROR) << "UVCStreamer: " << _device->id() << " does not support user pointer" << std::endl;
            return _initialized = false; // No usable capture mode available
          }
        }
        else
        {
          logger(INFO) << "UVCStreamer: " << _device->id() << " supports user pointer" << std::endl;
          _captureMode = CAPTURE_USER_POINTER;
          
          if(req.count < 2)
          {
            logger(ERROR) << "UVCStreamer: " << _device->id() << " insufficient buffers to capture" << std::endl;
            return _initialized = false;
          }
          
          _rawDataBuffers.resize(req.count);
        }
      }
      else
      {
        logger(ERROR) << "UVCStreamer: " << _device->id() << " VIDIC_REQBUFS failed" << std::endl;
        return _initialized = false;
      }
    }
    else
    {
      logger(INFO) << "UVCStreamer: " << _device->id() << " supports mmap" << std::endl;
      _captureMode = CAPTURE_MMAP;
      
      if(req.count < 2)
      {
        logger(ERROR) << "UVCStreamer: " << _device->id() << " insufficient buffers to capture" << std::endl;
        return _initialized = false;
      }
      
      _rawDataBuffers.resize(req.count);
    }
  }
  
  //// Initialize _rawDataBuffers
  if(_captureMode == CAPTURE_MMAP)
  {
    for(auto i = 0; i < _rawDataBuffers.size(); i++)
    {
      struct v4l2_buffer buf;
      
      memset(&buf, 0, sizeof(buf));
      
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      
      if(_uvc->getUVCPrivate().xioctl(VIDIOC_QUERYBUF, &buf) == -1)
      {
        logger(ERROR) << "UVCStreamer: Could not prepare raw data buffer '" << i << "'" << endl;
        return _initialized = false;
      }
      
      _rawDataBuffers[i].size = buf.length;
      
      if(!_uvc->getUVCPrivate().mmap(buf.m.offset, _rawDataBuffers[i]))
      {
        logger(ERROR) << "UVCStreamer: Failed to get raw memory mapped buffer '" << i << "'" << endl;
        return _initialized = false;
      }
    }
  }
  else if(_captureMode == CAPTURE_USER_POINTER)
  {
    for(auto i = 0; i < _rawDataBuffers.size(); i++)
    {
      _rawDataBuffers[i].size = _frameByteSize;
      _rawDataBuffers[i].data = Ptr<ByteType>(new ByteType[_frameByteSize], 
                                              [](ByteType *d){ delete []d; });
    }
  }
  
  /// Enqueue _rawDataBuffers and start streaming
  if(_captureMode == CAPTURE_MMAP || _captureMode == CAPTURE_USER_POINTER)
  {
    for(auto i = 0; i < _rawDataBuffers.size(); i++)
    {
      struct v4l2_buffer buf;
      
      memset(&buf, 0, sizeof(buf));
      
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = (_captureMode == CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
      buf.index  = i;
      
      if(_captureMode == CAPTURE_USER_POINTER)
      {
        buf.m.userptr = (unsigned long)&*_rawDataBuffers[i].data;
        buf.length = _rawDataBuffers[i].size;
      }
      
      if(_uvc->getUVCPrivate().xioctl(VIDIOC_QBUF, &buf) == -1)
      {
        logger(ERROR) << "UVCStreamer: Could not queue raw data buffer '" << i << "'" << endl;
        return _initialized = false;
      }
    }
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if(_uvc->getUVCPrivate().xioctl(VIDIOC_STREAMON, &type) == -1)
    {
      logger(ERROR) << "UVCStreamer: Failed to start capture stream" << endl;
      return _initialized = false;
    }
  }
#elif defined(WINDOWS)
#endif

  return true;
}

bool UVCStreamer::_stop()
{
  if(!isInitialized())
    return false;

#ifdef LINUX  
  /// Stop streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_STREAMOFF, &type) == -1)
  {
    logger(ERROR) << "UVCStreamer: Failed to stop capture stream" << endl;
    return _initialized = false;
  }
  
  /// Remove MMAPs if any
  _uvc->getUVCPrivate().clearMMap();
  
  /// Remove requestbuffers
  struct v4l2_requestbuffers req;
  
  memset(&req, 0, sizeof(req));
  
  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = (_captureMode == CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
  
  if(_uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
  {
    logger(ERROR) << "UVCStreamer: Failed to remove buffers" << endl;
    return _initialized = false;
  }
  
  /// Remove buffers
  _rawDataBuffers.clear();
#elif defined(WINDOWS)
#endif
  
  return true;
}

bool UVCStreamer::_capture(RawDataFramePtr &p)
{
  TimeStampType waitTime = 2000;//ms
  
#ifdef LINUX
  bool timedOut;
  if(!isInitialized() || !_uvc->getUVCPrivate().isReadReady(waitTime, timedOut))
  {
    if(timedOut)
      logger(ERROR) << "No data available. Waited for " << waitTime << " ms" << endl;
    
    return false;
  }
  
  if(_captureMode == CAPTURE_READ_WRITE)
  {
    if(!p || p->data.size() != _frameByteSize)
    {
      p = RawDataFramePtr(new RawDataFrame());
      p->data.resize(_frameByteSize);
      logger(DEBUG) << "UVCStreamer: Frame provided is not of appropriate size. Recreating a new frame." << endl;
    }
    
    bool ret = _uvc->read(p->data.data(), _frameByteSize);
    
    if(ret)
    {
      p->id = _currentID++;
      p->timestamp = _time.getCurentRealTime();
      return true;
    }
    return false;
  }
  else if(_captureMode == CAPTURE_MMAP || _captureMode == CAPTURE_USER_POINTER)
  {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = (_captureMode == CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
    
    if(_uvc->getUVCPrivate().xioctl(VIDIOC_DQBUF, &buf) == -1)
    {
      switch(errno)
      {
        case EIO:
          /* Could ignore EIO, see spec. */
          /* fall through */
          
        default:
          logger(ERROR) << "UVCStreamer: Failed to dequeue a raw frame buffer" << endl;
          return false;
      }
    }
    
    if(_captureMode == CAPTURE_USER_POINTER)
    {
      buf.index = _rawDataBuffers.size();
      for(auto i = 0; i < _rawDataBuffers.size(); i++)
        if(&*_rawDataBuffers[i].data == (ByteType *)buf.m.userptr && _rawDataBuffers[i].size == buf.bytesused)
        {
          buf.index = i;
          break;
        }
    }
    
    assert(buf.index < _rawDataBuffers.size());
    
    if(buf.bytesused < _frameByteSize)
    {
      logger(ERROR) << "Incomplete frame data. Skipping it." << endl;
      return false;
    }
    
    if(!p || p->data.size() != buf.bytesused)
    {
      p = RawDataFramePtr(new RawDataFrame());
      p->data.resize(buf.bytesused);
      logger(DEBUG) << "UVCStreamer: Frame provided is not of appropriate size. Recreating a new frame." << endl;
    }
    
    p->timestamp = _time.convertToRealTime(buf.timestamp.tv_sec*1000000L + buf.timestamp.tv_usec);
    memcpy(p->data.data(), &*_rawDataBuffers[buf.index].data, buf.bytesused);
    
    if(_uvc->getUVCPrivate().xioctl(VIDIOC_QBUF, &buf) == -1)
    {
      logger(ERROR) << "UVCStreamer: Failed to enqueue back the raw frame buffer" << endl;
      return false;
    }
    
    p->id = _currentID++;
    
    return true;
  }
#elif defined(WINDOWS)
  return false;
#endif
}

bool UVCStreamer::getSupportedVideoModes(Vector<VideoMode> &videoModes)
{
  if(!isInitialized())
    return false;
  
  videoModes.clear();
  
#ifdef LINUX
  int frameSizeIndex = 0;
  int frameRateIndex = 0;
  
  uint32_t frameWidth, frameHeight;
  
  
  struct v4l2_frmsizeenum frameSizeEnum;
  struct v4l2_frmivalenum frameIntervalEnum;
  
  while(1)
  {
    memset(&frameSizeEnum, 0, sizeof(frameSizeEnum));
    
    frameSizeEnum.index = frameSizeIndex++;
    frameSizeEnum.pixel_format = V4L2_PIX_FMT_YUYV;//V4L2_PIX_FMT_UYVY;//V4L2_PIX_FMT_YUYV;
    
    if(_uvc->getUVCPrivate().xioctl(VIDIOC_ENUM_FRAMESIZES, &frameSizeEnum) == -1)
    {
      if(errno == EINVAL)
        break;
      else
        return false;
    }
    
    if(frameSizeEnum.type != V4L2_FRMSIZE_TYPE_DISCRETE)
    {
      logger(WARNING) << "Frame types other than discrete, are not supported currently." << endl;
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
      
      if(_uvc->getUVCPrivate().xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &frameIntervalEnum) == -1)
      {
        if(errno == EINVAL)
          break;
        else
          return false;
      }
      
      if(frameIntervalEnum.type != V4L2_FRMIVAL_TYPE_DISCRETE)
      {
        logger(WARNING) << "Frame interval types other than discrete, are not supported currently." << endl;
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
#elif defined(WINDOWS)
#endif
  return true;
}

UVCStreamer::~UVCStreamer()
{
  if(isInitialized() && isRunning())
    stop(); // NOTE This cannot be called in the base class desctructor as UVCStreamer would already be destroyed by then
}

  
}