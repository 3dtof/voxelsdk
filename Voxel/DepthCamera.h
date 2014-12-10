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
  
  Map<String, ParameterPtr> _parameters;
  
  void _addParameters(const Vector<ParameterPtr> &params);
  
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
  DepthCamera(DevicePtr device): _device(device) {}
  
  virtual bool isInitialized() = 0;
  
  inline bool isRunning() { return _running; }
  
  virtual bool get(const String &name, bool &value, bool refresh = false);
  virtual bool get(const String &name, int &value, bool refresh = false); // works for int type and enum type
  virtual bool get(const String &name, float &value, bool refresh = false);
  
  virtual bool set(const String &name, bool value);
  virtual bool set(const String &name, int value); // works for int type and enum type
  virtual bool set(const String &name, float value);
  
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


typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
