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

#include <Filter/FilterSet.h>
#include "FrameStream.h"
#include "FrameGenerator.h"
#include "PointCloudFrameGenerator.h"
#include "Configuration.h"

#define CALIB_SECT_LENS "lens"
#define CALIB_SECT_LENS_ID 0

namespace Voxel
{
  
/**
  * \ingroup CamSys
  * 
  * \brief This is primary class which provides API for a depth camera. 
  * 
  * DepthCamera is an abstract class which needs to be derived and implemented 
  * for individual depth camera types.
  */
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
  
private:
  mutable Mutex _accessMutex; // This is locked by getters and setters which are public
  mutable Mutex _frameStreamWriterMutex;
  
protected:
  DevicePtr _device;
  
  String _name, _id, _chipset;
  
  Map<String, ParameterPtr> _parameters;
  
  Ptr<RegisterProgrammer> _programmer;
  Ptr<Streamer> _streamer;
  
  // NOTE: Constructors of derived classes need to initialize the first two entries in this array
  FrameGeneratorPtr _frameGenerators[3];
  
  Ptr<PointCloudFrameGenerator> _pointCloudFrameGenerator;
  
  bool _parameterInit;
  
  FrameBufferManager<RawFrame> _rawFrameBuffers;
  FrameBufferManager<DepthFrame> _depthFrameBuffers;
  FrameBufferManager<PointCloudFrame> _pointCloudBuffers;
  
  FilterSet<RawFrame> _unprocessedFilters, _processedFilters;
  
  FilterSet<DepthFrame> _depthFilters;
  
  FrameStreamWriterPtr _frameStreamWriter;
  
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
  
  bool _running, _isPaused; // is capture running?
  
  bool _writeToFrameStream(RawFramePtr &rawUnprocessed);
  
  // These protected getters and setters are not thread-safe. These are to be directly called only when nested calls are to be done from getter/setter to another. 
  // Otherwise use the public functions
  template <typename T>
  bool _get(const String &name, T &value, bool refresh = false) const;
  
  template <typename T>
  bool _set(const String &name, const T &value);
  
  virtual bool _setFrameRate(const FrameRate &r) = 0;
  virtual bool _getFrameRate(FrameRate &r) const = 0;
  
  virtual bool _setFrameSize(const FrameSize &s) = 0;
  virtual bool _getFrameSize(FrameSize &s) const = 0;
  virtual bool _getMaximumFrameSize(FrameSize &s) const = 0;
  virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const = 0;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const = 0;
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const = 0;
  
  virtual bool _getBytesPerPixel(uint &bpp) const = 0;
  virtual bool _setBytesPerPixel(const uint &bpp) = 0;
  
  
  virtual bool _getROI(RegionOfInterest &roi) const = 0;
  virtual bool _setROI(const RegionOfInterest &roi) = 0;
  virtual bool _allowedROI(String &message) = 0;
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const = 0;
  
  inline void _makeID() { _id = _name + "(" + _device->id() + ")"; }
  
  virtual bool _reset() = 0;
  virtual bool _onReset() = 0;
  
  virtual bool _applyConfigParams(const ConfigSet *params);
  
  virtual bool _saveCurrentProfileID(const int id) = 0;
  virtual bool _getCurrentProfileID(int &id) = 0;
  
  bool _init();
  
  inline Map<String, CalibrationInformation> &_getCalibrationInformationStructure() { return configFile._calibrationInformation; }
  
public:
  MainConfigurationFile configFile; // This corresponds to camera specific configuration file
  
  DepthCamera(const String &name, const String &chipset, DevicePtr device);
  
  virtual bool isInitialized() const
  {
    return _programmer && _programmer->isInitialized() && 
           _streamer && _streamer->isInitialized() && _parameterInit;
  }
  
  inline const String &name() const { return _name; }
  
  inline const String &id() const { return _id; }
  
  inline const String &chipset() const { return _chipset; }
  
  virtual bool getSerialNumber(String &serialNumber) const;
  virtual bool setSerialNumber(const String &serialNumber);
  
  inline const DevicePtr &getDevice() const { return _device; }
  
  inline bool isRunning() const { return _running; }
  
  inline bool isPaused() const { return _isPaused; }
  
  bool pause();
  bool resume();
  
  template <typename T>
  bool getStreamParam(const String &name, T &value) const;
  
  bool refreshParams();
  
  template <typename T>
  bool get(const String &name, T &value, bool refresh = false) const;
  
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
  inline bool getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
  inline bool getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  inline bool getMaximumVideoMode(VideoMode &videoMode) const;
  
  inline bool getBytesPerPixel(uint &bpp) const;
  inline bool setBytesPerPixel(const uint &bpp);
  
  inline bool getROI(RegionOfInterest &roi);
  inline bool setROI(const RegionOfInterest &roi);
  inline bool allowedROI(String &message);
  
  inline bool getFieldOfView(float &fovHalfAngle) const;
  
  virtual bool saveFrameStream(const String &fileName);
  virtual bool isSavingFrameStream();
  virtual bool closeFrameStream();
  
  virtual bool registerCallback(FrameType type, CallbackType f);
  virtual bool clearAllCallbacks();
  virtual bool clearCallback(FrameType type);
  
  // beforeFilterIndex = -1 => at the end, otherwise at location before the given filter index.
  // Return value: 
  //   >= 0 => add successfully with return value as filter ID.
  //     -1 => failed to add filter
  virtual int addFilter(FilterPtr p, FrameType frameType, int beforeFilterID = -1);
  virtual FilterPtr getFilter(int filterID, FrameType frameType) const;
  virtual bool removeFilter(int filterID, FrameType frameType);
  virtual bool removeAllFilters(FrameType frameType);
  virtual void resetFilters();
  
  inline const FilterSet<RawFrame> &getUnprocessedRawFilterSet() { return _unprocessedFilters; }
  inline const FilterSet<RawFrame> &getProcessedRawFilterSet() { return _processedFilters; }
  inline const FilterSet<DepthFrame> &getDepthFilterSet() { return _depthFilters; }
  
  bool start();
  bool stop();
  
  void wait();
  
  bool reset();
  
  inline Ptr<RegisterProgrammer> getProgrammer() { return _programmer; } // RegisterProgrammer is usually thread-safe to use outside directly
  inline Ptr<Streamer> getStreamer() { return _streamer; } // Streamer may not be thread-safe
  
  inline bool reloadConfiguration() { return configFile.read(_name + ".conf"); }
  inline const Map<int, String> &getCameraProfileNames() { return configFile.getCameraProfileNames(); }
  inline int getCurrentCameraProfileID() { return configFile.getCurrentProfileID(); }
  
  int addCameraProfile(const String &profileName, const int parentID);
  bool setCameraProfile(const int id, bool softApply = false);
  bool removeCameraProfile(const int id);
  inline bool saveCameraProfileToHardware(int &id, bool saveParents = false, bool setAsDefault = false, const String &namePrefix = "") { return configFile.saveCameraProfileToHardware(id, saveParents, setAsDefault, namePrefix); }
  
  bool close();
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

bool DepthCamera::getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
{
  Lock<Mutex> _(_accessMutex);
  return _getMaximumFrameRate(frameRate, forFrameSize);
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

bool DepthCamera::getBytesPerPixel(uint &bpp) const
{
  Lock<Mutex> _(_accessMutex);
  return _getBytesPerPixel(bpp);
}

bool DepthCamera::setBytesPerPixel(const uint &bpp)
{
  Lock<Mutex> _(_accessMutex);
  return _setBytesPerPixel(bpp);
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
      logger(LOG_ERROR) << "DepthCamera: Invalid value type '" << typeid(value).name() << "' used to get parameter " << _id << "." << name << std::endl;
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
    logger(LOG_DEBUG) << "DepthCamera: Setting parameter '" << name << "' = " << value << std::endl;
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

template <typename T>
bool DepthCamera::getStreamParam(const String &name, T &value) const
{
  if(!_frameGenerators[0]->get(name, value) && !_frameGenerators[1]->get(name, value) && !_frameGenerators[2]->get(name, value))
    return false;
  
  return true;
}


typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
