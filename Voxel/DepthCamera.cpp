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

bool DepthCamera::get(const String &name, int &value, bool refresh)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(IntegerParameter))
      return ((IntegerParameter &)*p->second).get(value, refresh);
    else if(typeid(*p->second) == typeid(EnumParameter))
      return ((EnumParameter &)*p->second).get(value, refresh);
    else
      return false;
  }
  else
    return false;
}

bool DepthCamera::get(const String &name, bool &value, bool refresh)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(BoolParameter))
      return ((BoolParameter &)*p->second).get(value, refresh);
    else
      return false;
  }
  else
    return false;
}

bool DepthCamera::get(const String &name, float &value, bool refresh)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(FloatParameter))
      return ((FloatParameter &)*p->second).get(value, refresh);
    else
      return false;
  }
  else
    return false;
}

bool DepthCamera::set(const String &name, int value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(IntegerParameter))
      return ((IntegerParameter &)*p->second).set(value);
    else if(typeid(*p->second) == typeid(EnumParameter))
      return ((EnumParameter &)*p->second).set(value);
    else
      return false;
  }
  else
    return false;
}

bool DepthCamera::set(const String &name, bool value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(BoolParameter))
      return ((BoolParameter &)*p->second).set(value);
    else
      return false;
  }
  else
    return false;
}

bool DepthCamera::set(const String &name, float value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    if(typeid(*p->second) == typeid(FloatParameter))
      return ((FloatParameter &)*p->second).set(value);
    else
      return false;
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