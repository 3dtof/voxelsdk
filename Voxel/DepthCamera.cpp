/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif


#include "DepthCamera.h"
#include "Logger.h"

namespace Voxel
{
  
bool DepthCamera::_addParameters(const Vector<ParameterPtr> &params)
{
  _parameters.reserve(_parameters.size() + params.size());
  
  for(const ParameterPtr &p: params)
  {
    if(_parameters.find(p->name()) == _parameters.end())
    {
      _parameters[p->name()] = p;
    }
    else
    {
      logger(LOG_ERROR) << "DepthCamera: Found an existing parameter in the list of parameters, with name " << p->name() << ". Not overwriting it." << endl;
      return _parameterInit = false;
    }
  }
  return true;
}

bool DepthCamera::clearCallback()
{
  for(auto i = 0; i < CALLBACK_TYPE_COUNT; i++)
    _callback[i] = nullptr;
  return true;
}

bool DepthCamera::registerCallback(FrameCallBackType type, CallbackType f)
{
  if(type < CALLBACK_TYPE_COUNT)
  {
    if(_callback[type])
      logger(LOG_WARNING) << "DepthCamera: " << id() << " already has a callback for this type = " << type << ". Overwriting it now." << std::endl;
    
    _callBackTypesRegistered |= (1 << type);
    _callback[type] = f;
    return true;
  }
  logger(LOG_ERROR) << "DepthCamera: Invalid callback type = " << type << " attempted for depth camera " << id() << std::endl;
  return false;
}

bool DepthCamera::_callbackAndContinue(uint32_t &callBackTypesToBeCalled, DepthCamera::FrameCallBackType type, const Frame &frame)
{
  if((callBackTypesToBeCalled | type) && _callback[type])
  {
    _callback[type](*this, frame, type);
  }
  
  callBackTypesToBeCalled &= ~type;
  
  return callBackTypesToBeCalled != 0;
}


void DepthCamera::_captureLoop()
{
  uint consecutiveCaptureFails = 0;
  
  while(_running)
  {
    uint32_t callBackTypesToBeCalled = _callBackTypesRegistered;
    
    if(consecutiveCaptureFails > 100)
    {
      logger(LOG_ERROR) << "DepthCamera: 100 consecutive failures in capture of frame. Stopping stream for " << id() << std::endl;
      _running = false;
      continue;
    }
    
    if(_callBackTypesRegistered == 0 && _callBackTypesRegistered == CALLBACK_RAW_FRAME_UNPROCESSED) // Only unprocessed frame types requested or none requested?
    {
      auto f = _rawFrameBuffers.get();
      
      if(!_captureRawUnprocessedFrame(*f))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      if(_callback[CALLBACK_RAW_FRAME_UNPROCESSED])
        _callback[CALLBACK_RAW_FRAME_UNPROCESSED](*this, (Frame &)(**f), CALLBACK_RAW_FRAME_UNPROCESSED);
      consecutiveCaptureFails = 0;
    }
    else
    {
      RawFramePtr f1;
      if(!_captureRawUnprocessedFrame(f1))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, CALLBACK_RAW_FRAME_UNPROCESSED, *f1))
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto f = _rawFrameBuffers.get();
      
      if(!_processRawFrame(f1, *f))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, CALLBACK_RAW_FRAME_PROCESSED, **f))
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto d = _depthFrameBuffers.get();
      
      if(!_convertToDepthFrame(*f, *d))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, CALLBACK_DEPTH_FRAME, **d))
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto p = _pointCloudBuffers.get();
      
      if(!_convertToPointCloudFrame(*d, *p))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      _callbackAndContinue(callBackTypesToBeCalled, CALLBACK_XYZI_POINT_CLOUD_FRAME, **p);
      consecutiveCaptureFails = 0;
    }
  }
  
  if(!_running)
  {
    _stop();
  }
}

bool DepthCamera::_convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame)
{
  if(!depthFrame)
  {
    logger(LOG_ERROR) << "DepthCamera: Blank depth frame." << std::endl;
    return false;
  }
  
  XYZIPointCloudFrame *f = dynamic_cast<XYZIPointCloudFrame *>(pointCloudFrame.get());
  
  if(!f)
  {
    f = new XYZIPointCloudFrame();
    pointCloudFrame = PointCloudFramePtr(f);
  }
  
  f->id = depthFrame->id;
  f->timestamp = depthFrame->timestamp;
  f->points.resize(depthFrame->size.width*depthFrame->size.height);
  
  if(!_pointCloudTransform->depthToPointCloud(depthFrame->depth, *pointCloudFrame))
  {
    logger(LOG_ERROR) << "DepthCamera: Could not convert depth frame to point cloud frame" << std::endl;
    return false;
  }
  
  // Setting amplitude as intensity
  auto index = 0;
  
  auto w = depthFrame->size.width;
  auto h = depthFrame->size.height;
  
  for(auto y = 0; y < h; y++)
    for(auto x = 0; x < w; x++, index++)
    {
      IntensityPoint &p = f->points[index];
      p.i = depthFrame->amplitude[index];
    }
  
  /*
  auto index = 0;
  
  auto x1 = 0, y1 = 0;
  
  auto theta = 0.0f, phi = 0.0f, thetaMax = 0.0f;
  
  auto w = depthFrame->size.width;
  auto h = depthFrame->size.height;
  
  auto scaleMax = sqrt(w*w/4.0f + h*h/4.0f);
  
  if(!getFieldOfView(thetaMax) || thetaMax == 0)
  {
    logger(LOG_ERROR) << "DepthCamera: Could not get the field of view angle for " << id() << std::endl;
    return false;
  }
  
  float focalLength;
  
  focalLength = scaleMax/tan(thetaMax);
  
  auto r = 0.0f;
  
  for(auto y = 0; y < h; y++)
    for(auto x = 0; x < w; x++, index++)
    {
      IntensityPoint &p = f->points[index];
      
      x1 = x - w/2;
      y1 = y - h/2;
      
      phi = atan(y1*1.0/x1);
      
      if(x1 < 0)
        phi = M_PI + phi; // atan() principal range [-PI/2, PI/2]. outside that add PI
      
      theta = atan(sqrt(x1*x1 + y1*y1)/focalLength);
      
      r = depthFrame->depth[index];
      p.i = depthFrame->amplitude[index];
      
      p.x = r*sin(theta)*cos(phi);
      p.y = r*sin(theta)*sin(phi);
      p.z = r*cos(theta);
      
      //logger(INFO) << "Point = " << p.i << "@(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }
    */
  return true;
}


void DepthCamera::_captureThreadWrapper()
{
  _captureLoop();
}

bool DepthCamera::start()
{
  if(!_callback)
  {
    logger(LOG_ERROR) << "DepthCamera: Please register a callback to " << _id << " before starting capture" << std::endl;
    return false;
  }
  
  if(!_start())
    return false;
  
  _running = true;
  
  wait();
  //_captureThreadWrapper();
  _captureThread = ThreadPtr(new Thread(&DepthCamera::_captureThreadWrapper, this));
  
  return true;
}

bool DepthCamera::stop()
{
  _running = false;
  wait();
  return true;
}

void DepthCamera::wait()
{
  if(_captureThread &&  _captureThread->get_id() != std::this_thread::get_id() && _captureThread->joinable())
    _captureThread->join();
}

DepthCamera::~DepthCamera()
{
  stop();
  
  _rawFrameBuffers.clear();
  _depthFrameBuffers.clear();
  _pointCloudBuffers.clear();
  
  _parameters.clear();
}

bool DepthCamera::reset()
{
  if(!stop())
    return false;
  
  if(!_programmer->reset())
  {
    logger(LOG_ERROR) << "DepthCamera: Failed to reset device " << id() << std::endl;
    return false;
  }
  _programmer = nullptr;
  _streamer = nullptr;
  return true;
}


  
}