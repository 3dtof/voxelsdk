/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEPTHCAMERA_H
#define VOXEL_DEPTHCAMERA_H

#include <Device.h>
#include <Parameter.h>
#include <Frame.h>

namespace Voxel
{
  
class DepthCamera
{
protected:
  DevicePtr _device;
  
  String _name;
  
  Map<String, ParameterPtr> _parameters;
  
  bool _addParameters(const Vector<ParameterPtr> &params);
  
  typedef Function<void (DepthCamera &camera, DepthFramePtr frame)> DepthFrameCallbackType;
  DepthFrameCallbackType _depthFrameCallbackType;
  
  typedef Function<void (DepthCamera &camera, XYZPointCloudFramePtr frame)> XYZPointCloudFrameCallbackType;
  XYZPointCloudFrameCallbackType _xyzPointCloudFrameCallbackType;
  
  typedef Function<void (DepthCamera &camera, RawFramePtr frame)> RawFrameCallbackType;
  RawFrameCallbackType _rawFrameCallbackType;
  
  ThreadPtr _captureThread;
  
  virtual bool _start() = 0;
  virtual bool _stop() = 0;
  
  virtual bool _captureRawFrame(RawFramePtr &rawFrame) = 0;
  virtual bool _captureDepthFrame(RawFramePtr &rawFrame, DepthFramePtr &depthFrame) = 0;
  
  virtual void _captureLoop(); // the main capture loop
  
  void _captureThreadWrapper(); // this is non-virtual and simply calls _captureLoop
  
  bool _running; // is capture running?
  
public:
  DepthCamera(const String &name, DevicePtr device): _device(device), _name(name) {}
  
  virtual bool isInitialized() = 0;
  
  inline const String &name() { return _name; }
  
  inline bool isRunning() { return _running; }
  
  template <typename T>
  bool get(const String &name, T &value, bool refresh = false);
  
  template <typename T>
  bool set(const String &name, const T &value);
  
  virtual bool registerCallback(DepthFrameCallbackType f);
  virtual bool registerCallback(XYZPointCloudFrameCallbackType f);
  virtual bool registerCallback(RawFrameCallbackType f);
  
  virtual bool clearDepthFrameCallback();
  virtual bool clearXYZPointCloudFrameCallback();
  virtual bool clearRawFrameCallback();
  
  virtual bool start();
  virtual bool stop();
  
  virtual void wait();
  
  virtual ~DepthCamera() {}
};

template <typename T>
bool DepthCamera::get(const String &name, T &value, bool refresh)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());
    
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
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());
    
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

typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
