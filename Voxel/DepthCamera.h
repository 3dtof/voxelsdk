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
#include "VideoMode.h"
#include <FrameBuffer.h>

#include <RegisterProgrammer.h>
#include <Streamer.h>

#include <PointCloudTransform.h>

#include <Filter/FrameFilterSet.h>


#define MAX_FRAME_BUFFERS 2

namespace Voxel
{
  
class VOXEL_EXPORT DepthCamera
{
public:
  enum FrameType
  {
    FRAME_RAW_FRAME_UNPROCESSED = 0,
    FRAME_RAW_FRAME_PROCESSED = 1,
    FRAME_DEPTH_FRAME = 2,
    FRAME_XYZI_POINT_CLOUD_FRAME = 3,
    FRAME_TYPE_COUNT = 4 // This is just used for number of callback types
  };
  
  typedef Function<void (DepthCamera &camera, const Frame &frame, FrameType callBackType)> CallbackType;
  
protected:
  DevicePtr _device;
  
  String _name, _id;
  
  Map<String, ParameterPtr> _parameters;
  
  Ptr<RegisterProgrammer> _programmer;
  Ptr<Streamer> _streamer;
  Ptr<PointCloudTransform> _pointCloudTransform;
  
  bool _parameterInit;
  
  FrameBufferManager<RawFrame> _rawFrameBuffers;
  FrameBufferManager<DepthFrame> _depthFrameBuffers;
  FrameBufferManager<PointCloudFrame> _pointCloudBuffers;
  
  FrameFilterSet<RawFrame> _unprocessedFrameFilters, _processedFrameFilters;
  
  FrameFilterSet<DepthFrame> _depthFrameFilters;
  
  bool _addParameters(const Vector<ParameterPtr> &params);
  
  CallbackType _callback[FRAME_TYPE_COUNT];
  
  uint32_t _callBackTypesRegistered = 0;
  
  ThreadPtr _captureThread;
  
  // Callback the registered function for 'type' if present and decide whether continue processing or not
  virtual bool _callbackAndContinue(uint32_t &callBackTypesToBeCalled, FrameType type, const Frame &frame);
  
  virtual bool _start() = 0;
  virtual bool _stop() = 0;
  
  virtual bool _captureRawUnprocessedFrame(RawFramePtr &rawFrame) = 0;
  virtual bool _processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput) = 0; // here output raw frame will have processed data, like ToF data for ToF cameras
  virtual bool _convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame) = 0;
  virtual bool _convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame);
  
  virtual void _captureLoop(); // the main capture loop
  
  void _captureThreadWrapper(); // this is non-virtual and simply calls _captureLoop
  
  bool _running; // is capture running?
  
  mutable Mutex _accessMutex; // This is locked by getters and setters which are public
  
  // These protected getters and setters are not thread-safe. These are to be directly called only when nested calls are to be done from getter/setter to another. 
  // Otherwise use the public functions
  template <typename T>
  bool _get(const String &name, T &value, bool refresh = true) const;
  
  template <typename T>
  bool _set(const String &name, const T &value);
  
  virtual bool _setFrameRate(const FrameRate &r) = 0;
  virtual bool _getFrameRate(FrameRate &r) const = 0;
  
  virtual bool _setFrameSize(const FrameSize &s) = 0;
  virtual bool _getFrameSize(FrameSize &s) const = 0;
  virtual bool _getMaximumFrameSize(FrameSize &s) const = 0;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const = 0;
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const = 0;
  
  virtual bool _getROI(RegionOfInterest &roi) = 0;
  virtual bool _setROI(const RegionOfInterest &roi) = 0;
  virtual bool _allowedROI(String &message) = 0;
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const = 0;
  
  inline void _makeID() { _id = _name + "(" + _device->id() + ")"; }
  
public:
  DepthCamera(const String &name, DevicePtr device): _device(device), _name(name),
  _rawFrameBuffers(MAX_FRAME_BUFFERS), _depthFrameBuffers(MAX_FRAME_BUFFERS), _pointCloudBuffers(MAX_FRAME_BUFFERS),
  _parameterInit(true), _running(false),
  _unprocessedFrameFilters(_rawFrameBuffers), _processedFrameFilters(_rawFrameBuffers),
  _depthFrameFilters(_depthFrameBuffers)
  {
    _makeID();
  }
  
  virtual bool isInitialized() const
  {
    return _programmer && _programmer->isInitialized() && 
           _streamer && _streamer->isInitialized() && _parameterInit;
  }
  
  inline const String &name() const { return _name; }
  
  inline const String &id() const { return _id; }
  
  inline const DevicePtr &getDevice() const { return _device; }
  
  inline bool isRunning() const { return _running; }
  
  template <typename T>
  bool get(const String &name, T &value, bool refresh = true) const;
  
  template <typename T>
  bool set(const String &name, const T &value);
  
  // WARNING: Avoid using get() and set() on ParameterPtr, obtained via getParam() or getParameters(). It is not thread-safe. Instead use get() and set() on DepthCamera
  inline const ParameterPtr getParam(const String &name) const;
  inline const Map<String, ParameterPtr> &getParameters() const { return _parameters; }
  
  inline bool setFrameRate(const FrameRate &r);
  inline bool getFrameRate(FrameRate &r) const;
  
  inline bool setFrameSize(const FrameSize &s);
  inline bool getFrameSize(FrameSize &s) const;
  inline bool getMaximumFrameSize(FrameSize &s) const;
  inline bool getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  inline bool getMaximumVideoMode(VideoMode &videoMode) const;
  
  inline bool getROI(RegionOfInterest &roi);
  inline bool setROI(const RegionOfInterest &roi);
  inline bool allowedROI(String &message);
  
  inline bool getFieldOfView(float &fovHalfAngle) const;
  
  virtual bool registerCallback(FrameType type, CallbackType f);
  virtual bool clearCallback();
  
  virtual int addFilter(FrameFilterPtr p, FrameType frameType);
  virtual bool removeFilter(int filterID, FrameType frameType);
  virtual bool removeAllFilters(FrameType frameType);
  
  bool start();
  bool stop();
  
  void wait();
  
  virtual bool reset();
  
  inline Ptr<RegisterProgrammer> getProgrammer() { return _programmer; } // RegisterProgrammer is usually thread-safe to use outside directly
  inline Ptr<Streamer> getStreamer() { return _streamer; } // Streamer may not be thread-safe
  
  virtual ~DepthCamera();
};

template <typename T>
bool DepthCamera::get(const String &name, T &value, bool refresh) const
{
  Lock<Mutex> _(_accessMutex);
  return _get(name, value, refresh);
}

template <typename T>
bool DepthCamera::set(const String &name, const T &value)
{
  Lock<Mutex> _(_accessMutex);
  return _set(name, value);
}

bool DepthCamera::getFieldOfView(float &fovHalfAngle) const
{
  Lock<Mutex> _(_accessMutex);
  return _getFieldOfView(fovHalfAngle);
}

bool DepthCamera::getFrameRate(FrameRate &r) const
{
  Lock<Mutex> _(_accessMutex);
  return _getFrameRate(r);
}

bool DepthCamera::setFrameRate(const FrameRate &r)
{
  Lock<Mutex> _(_accessMutex);
  return _setFrameRate(r);
}

bool DepthCamera::getFrameSize(FrameSize &s) const
{
  Lock<Mutex> _(_accessMutex);
  return _getFrameSize(s);
}

bool DepthCamera::setFrameSize(const FrameSize &s)
{
  Lock<Mutex> _(_accessMutex);
  return _setFrameSize(s);
}

bool DepthCamera::getMaximumFrameSize(FrameSize &s) const
{
  Lock<Mutex> _(_accessMutex);
  return _getMaximumFrameSize(s);
}

bool DepthCamera::getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
{
  Lock<Mutex> _(_accessMutex);
  return _getSupportedVideoModes(supportedVideoModes);
}

bool DepthCamera::getMaximumVideoMode(VideoMode &videoMode) const
{
  Lock<Mutex> _(_accessMutex);
  return _getMaximumVideoMode(videoMode);
}


bool DepthCamera::allowedROI(String &message)
{
  Lock<Mutex> _(_accessMutex);
  return _allowedROI(message);
}

bool DepthCamera::getROI(RegionOfInterest &roi)
{
  Lock<Mutex> _(_accessMutex);
  return _getROI(roi);
}

bool DepthCamera::setROI(const RegionOfInterest &roi)
{
  Lock<Mutex> _(_accessMutex);
  return _setROI(roi);
}

template <typename T>
bool DepthCamera::_get(const String &name, T &value, bool refresh) const
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());
    
    if(param == 0)
    {
      logger(LOG_ERROR) << "DepthCamera: Invalid value type '" << typeid(value).name() << "' used to set parameter " << _id << "." << name << std::endl;
      return false;
    }
    
    if(!param->get(value, refresh))
    {
      logger(LOG_ERROR) << "DepthCamera:Could not get value for parameter " << _id << "." << name << std::endl;
      return false;
    }
    
    return true;
  }
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
    return false;
  }
}

template <typename T>
bool DepthCamera::_set(const String &name, const T &value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());
    
    if(param == 0)
    {
      logger(LOG_ERROR) << "DepthCamera: Invalid value type '" << typeid(value).name() << "' used to set parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    if(!param->set(value))
    {
      logger(LOG_ERROR) << "DepthCamera: Could not set value " << value << " for parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
      return false;
    }
    
    return true;
  }
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
    return false;
  }
}

const ParameterPtr DepthCamera::getParam(const String &name) const
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
    return p->second;
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
    return 0;
  }
}


typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
