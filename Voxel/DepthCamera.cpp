/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

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
      logger(ERROR) << "DepthCamera: Found an existing parameter in the list of parameters, with name " << p->name() << ". Not overwriting it." << endl;
      return false;
    }
  }
  return true;
}

bool DepthCamera::clearCallback()
{
  _callback = 0;
}

bool DepthCamera::registerCallback(FrameCallBackType type, CallbackType f)
{
  _callback = f;
  _callBackType = type;
}


void DepthCamera::_captureLoop()
{
  while(_running)
  {
    if(_callBackType == CALLBACK_NONE or _callBackType == CALLBACK_RAW_FRAME_UNPROCESSED)
    {
      auto f = _rawFrameBuffers.get();
      if(_captureRawUnprocessedFrame(*f) and _callback)
        _callback(*this, (Frame &)(**f), _callBackType);
    }
    else
    {
      RawFramePtr f1;
      if(!_captureRawUnprocessedFrame(f1))
        continue;
      
      auto f = _rawFrameBuffers.get();
      
      if(!_processRawFrame(f1, *f))
        continue;

      if(_callBackType == CALLBACK_RAW_FRAME_PROCESSED)
      {
        _callback(*this, (Frame &)(**f), _callBackType);
        continue;
      }
      
      
      auto d = _depthFrameBuffers.get();
      
      if(!_convertToDepthFrame(*f, *d))
        continue;
      
      if(_callBackType == CALLBACK_DEPTH_FRAME)
      {
        _callback(*this, (Frame &)(**d), _callBackType);
        continue;
      }
      
      auto p = _pointCloudBuffers.get();
      
      if(!_convertToPointCloudFrame(*d, *p))
      {
        continue;
      }
      
      _callback(*this, (Frame &)(**p), _callBackType); // point cloud type callback
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
    logger(ERROR) << "DepthCamera: Blank depth frame." << std::endl;
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
  
  auto index = 0;
  
  auto x1 = 0, y1 = 0;
  
  auto theta = 0.0f, phi = 0.0f, thetaMax = 0.0f;
  
  auto w = depthFrame->size.width;
  auto h = depthFrame->size.height;
  
  auto scaleMax = sqrt(w*w/4.0f + h*h/4.0f);
  
  if(!_getFieldOfView(thetaMax))
    return false;
  
  auto r = 0.0f;
  
  for(auto y = 0; y < h; y++)
    for(auto x = 0; x < w; x++, index++)
    {
      IntensityPoint &p = f->points[index];
      
      x1 = x - w/2;
      y1 = y - h/2;
      
      phi = tan(y1*1.0/x1);
      
      theta = sqrt(x1*x1 + y1*y1)/scaleMax*thetaMax;
      
      r = depthFrame->depth[index];
      p.i = depthFrame->amplitude[index];
      
      p.x = r*sin(theta)*cos(phi);
      p.y = r*sin(theta)*sin(phi);
      p.z = r*cos(theta);
    }
    
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
    logger(ERROR) << "DepthCamera: Please register a callback to " << _id << " before starting capture" << std::endl;
    return false;
  }
  
  if(!_start())
    return false;
  
  _running = true;
  
  //_captureThreadWrapper();
  _captureThread = ThreadPtr(new Thread(&DepthCamera::_captureThreadWrapper, this));
  
  return true;
}

bool DepthCamera::stop()
{
  _running = false;
  return true;
}

void DepthCamera::wait()
{
  if(isRunning())
    _captureThread->join();
}

DepthCamera::~DepthCamera()
{
  _rawFrameBuffers.clear();
  _depthFrameBuffers.clear();
  _pointCloudBuffers.clear();
  
  _parameters.clear();
}

  
}