/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DepthCamera.h"
#include "Logger.h"

namespace Voxel
{
  
void DepthCamera::_addParameters(const Vector<ParameterPtr> &params)
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
      log(WARNING) << "DepthCamera: Found an existing parameter in the list of parameters, with name " << p->name() << ". Not overwriting it." << endl;
    }
  }
}

bool DepthCamera::registerCallback(XYZPointCloudFrameCallbackType f)
{
  _xyzPointCloudFrameCallbackType = f;
}

bool DepthCamera::registerCallback(DepthFrameCallbackType f)
{
  _depthFrameCallbackType = f;
}

bool DepthCamera::registerCallback(RawFrameCallbackType f)
{
  _rawFrameCallbackType = f;
}

bool DepthCamera::clearXYZPointCloudFrameCallback()
{
  _xyzPointCloudFrameCallbackType = 0;
}

bool DepthCamera::clearDepthFrameCallback()
{
  _depthFrameCallbackType = 0;
}

bool DepthCamera::clearRawFrameCallback()
{
  _rawFrameCallbackType = 0;
}

template <typename T>
bool DepthCamera::get(const String &name, T &value, bool refresh)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T>>(p->second.get());
    
    if(param == 0)
    {
      log(ERROR) << "Invalid value type '" << typeid(value).name() << "' used to set parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    if(!param->get(value, refresh))
    {
      log(ERROR) << "Could not get value for parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    return true;
  }
  else
    return false;
}

template <typename T>
bool DepthCamera::set(const String &name, const T &value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T>>(p->second.get());
    
    if(param == 0)
    {
      log(ERROR) << "Invalid value type '" << typeid(value).name() << "' used to set parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    if(!param->set(value))
    {
      log(ERROR) << "Could not set value " << value << " for parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    return true;
  }
  else
    return false;
}

void DepthCamera::_captureLoop()
{
  while(_running)
  {
    RawFramePtr rawFrame;
    if(_captureRawFrame(rawFrame) && _rawFrameCallbackType)
      _rawFrameCallbackType(*this, rawFrame);
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

  
}