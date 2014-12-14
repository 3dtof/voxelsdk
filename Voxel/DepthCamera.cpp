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
    if(_callBackType == CALLBACK_RAW_FRAME_UNPROCESSED)
    {
      RawFramePtr &f = _rawFrameBuffers.get();
      if(_captureRawFrame(f))
        _callback(*this, (Frame &)(*f), _callBackType);
    }
  }
  
  if(!_running)
  {
    _stop();
  }
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