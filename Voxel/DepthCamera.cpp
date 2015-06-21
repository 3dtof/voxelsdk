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
#include "PointCloudFrameGenerator.h"

namespace Voxel
{
  
DepthCamera::DepthCamera(const String &name, DevicePtr device): _device(device), _name(name),
_rawFrameBuffers(MAX_FRAME_BUFFERS), _depthFrameBuffers(MAX_FRAME_BUFFERS), _pointCloudBuffers(MAX_FRAME_BUFFERS),
_parameterInit(true), _running(false),
_unprocessedFilters(_rawFrameBuffers), _processedFilters(_rawFrameBuffers), _depthFilters(_depthFrameBuffers),
_pointCloudFrameGenerator(new PointCloudFrameGenerator())
{
  _frameGenerators[2] = std::dynamic_pointer_cast<FrameGenerator>(_pointCloudFrameGenerator);
  _makeID();
  
  configFile.read(name + ".conf"); // Read and keep the configuration ready for use. The file must be in VOXEL_CONF_PATH
}

bool DepthCamera::_init()
{
  return setCameraProfile(configFile.getDefaultCameraProfileName());
}
  
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
      logger(LOG_ERROR) << "DepthCamera: Found an existing parameter in the list of parameters, with name " << p->name() << ". Not overwriting it." << std::endl;
      //return _parameterInit = false;
    }
  }
  return true;
}

bool DepthCamera::clearAllCallbacks()
{
  for(auto i = 0; i < FRAME_TYPE_COUNT; i++)
    _callback[i] = nullptr;
  return true;
}

bool DepthCamera::clearCallback(FrameType type)
{
  if(type < FRAME_TYPE_COUNT)
  {
    _callback[type] = nullptr;
    return true;
  }
  return false;
}

bool DepthCamera::registerCallback(FrameType type, CallbackType f)
{
  if(type < FRAME_TYPE_COUNT)
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

bool DepthCamera::_callbackAndContinue(uint32_t &callBackTypesToBeCalled, DepthCamera::FrameType type, const Frame &frame)
{
  if((callBackTypesToBeCalled | (1 << type)) && _callback[type])
  {
    _callback[type](*this, frame, type);
  }
  
  callBackTypesToBeCalled &= ~(1 << type);
  
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
    
    if((_callBackTypesRegistered == 0 || _callBackTypesRegistered == FRAME_RAW_FRAME_UNPROCESSED) && !isSavingFrameStream()) // Only unprocessed frame types requested or none requested?
    {
      auto f = _rawFrameBuffers.get();
      
      if(!_captureRawUnprocessedFrame(*f))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      if(_callback[FRAME_RAW_FRAME_UNPROCESSED])
      {
        FilterSet<RawFrame>::FrameSequence _frameBuffers;
        _frameBuffers.push_front(f);
        
        if(!_unprocessedFilters.applyFilter(_frameBuffers))
        {
          logger(LOG_ERROR) << "DepthCamera: Failed to apply filters on raw unprocessed frame" << std::endl;
          consecutiveCaptureFails++;
          continue;
        }
        
        _callback[FRAME_RAW_FRAME_UNPROCESSED](*this, (Frame &)(**_frameBuffers.begin()), FRAME_RAW_FRAME_UNPROCESSED);
        
        _writeToFrameStream(**_frameBuffers.begin());
      }
      else
      {
        _writeToFrameStream(*f);
      }
      
      consecutiveCaptureFails = 0;
    }
    else
    {
      auto f1 = _rawFrameBuffers.get();
      if(!_captureRawUnprocessedFrame(*f1))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      FilterSet<RawFrame>::FrameSequence _unprocessedFrameBuffers;
      _unprocessedFrameBuffers.push_front(f1);
      
      if(!_unprocessedFilters.applyFilter(_unprocessedFrameBuffers))
      {
        logger(LOG_ERROR) << "DepthCamera: Failed to apply filters on raw unprocessed frame" << std::endl;
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, FRAME_RAW_FRAME_UNPROCESSED, ***_unprocessedFrameBuffers.begin()) && !isSavingFrameStream())
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto f = _rawFrameBuffers.get();
      
      if(!_processRawFrame(**_unprocessedFrameBuffers.begin(), *f))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      FilterSet<RawFrame>::FrameSequence _processedFrameBuffers;
      _processedFrameBuffers.push_front(f);
      
      if(!_processedFilters.applyFilter(_processedFrameBuffers))
      {
        logger(LOG_ERROR) << "DepthCamera: Failed to apply filters on raw processed frame" << std::endl;
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, FRAME_RAW_FRAME_PROCESSED, ***_processedFrameBuffers.begin())  && !isSavingFrameStream())
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto d = _depthFrameBuffers.get();
      
      if(!_convertToDepthFrame(**_processedFrameBuffers.begin(), *d))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      FilterSet<DepthFrame>::FrameSequence _depthFrameBuffers;
      _depthFrameBuffers.push_front(d);
      
      if(!_depthFilters.applyFilter(_depthFrameBuffers))
      {
        logger(LOG_ERROR) << "DepthCamera: Failed to apply filters on depth frame" << std::endl;
        consecutiveCaptureFails++;
        continue;
      }
      
      if(!_callbackAndContinue(callBackTypesToBeCalled, FRAME_DEPTH_FRAME, ***_depthFrameBuffers.begin()) && !isSavingFrameStream())
      {
        consecutiveCaptureFails = 0;
        continue;
      }
      
      auto p = _pointCloudBuffers.get();
      
      if(!_convertToPointCloudFrame(**_depthFrameBuffers.begin(), *p))
      {
        consecutiveCaptureFails++;
        continue;
      }
      
      _callbackAndContinue(callBackTypesToBeCalled, FRAME_XYZI_POINT_CLOUD_FRAME, **p);
      consecutiveCaptureFails = 0;
      
      _writeToFrameStream(**_unprocessedFrameBuffers.begin());
    }
  }
  
  if(!_running)
  {
    closeFrameStream();
    _stop();
  }
  
  logger(LOG_INFO) << "DepthCamera: Streaming stopped." << std::endl;
}

bool DepthCamera::_convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame)
{
  if(!depthFrame)
  {
    logger(LOG_ERROR) << "DepthCamera: Blank depth frame." << std::endl;
    return false;
  }
  
  FramePtr p1 = std::dynamic_pointer_cast<Frame>(depthFrame);
  FramePtr p2 = std::dynamic_pointer_cast<Frame>(pointCloudFrame);
  
  bool ret = _pointCloudFrameGenerator->generate(p1, p2);
  
  if(ret)
  {
    pointCloudFrame = std::dynamic_pointer_cast<PointCloudFrame>(p2);
    return true;
  }
  else
    return false;
}


void DepthCamera::_captureThreadWrapper()
{
  _captureLoop();
}

bool DepthCamera::start()
{
  if (isRunning())
  {
    logger(LOG_ERROR) << "DepthCamera: Camera is already running. Please stop it before calling start() again." << std::endl;
    return false;
  }

  wait();

  if (_captureThread &&  _captureThread->joinable())
  {
    logger(LOG_ERROR) << "DepthCamera: Camera is still running. Please wait for the current capture loop to complete before calling start() again." << std::endl;
    return false;
  }

  if(!_callBackTypesRegistered)
  {
    logger(LOG_ERROR) << "DepthCamera: Please register a callback to " << _id << " before starting capture" << std::endl;
    return false;
  }
  
  resetFilters();
  
  if(!_start())
    return false;

  _running = true;
  //_captureThreadWrapper();
  _captureThread = ThreadPtr(new Thread(&DepthCamera::_captureThreadWrapper, this));
  
  return true;
}

bool DepthCamera::stop()
{
  if (!isRunning())
  {
    logger(LOG_WARNING) << "DepthCamera: Camera is not running." << std::endl;
    return true;
  }

  _running = false;
  wait();
  return true;
}

void DepthCamera::wait()
{
  if(_captureThread &&  _captureThread->get_id() != std::this_thread::get_id() && _captureThread->joinable())
    _captureThread->join();
}

bool DepthCamera::close()
{
  if(isRunning())
    stop();
  
  _programmer.reset();
  _streamer.reset();
  
  _rawFrameBuffers.clear();
  _depthFrameBuffers.clear();
  _pointCloudBuffers.clear();
  
  _parameters.clear();
  
  return true;
}


DepthCamera::~DepthCamera()
{
  close();
}

bool DepthCamera::reset()
{
  if(isRunning())
  {
    logger(LOG_ERROR) << "DepthCamera: Please stop the depth camera before calling reset" << std::endl;
    return false;
  }
  
  if(!_reset())
  {
    logger(LOG_ERROR) << "DepthCamera: Failed to reset device " << id() << std::endl;
    return false;
  }
  
  resetFilters();
  return true;
}

int DepthCamera::addFilter(FilterPtr p, DepthCamera::FrameType frameType, int beforeFilterID)
{
  if(frameType == FRAME_RAW_FRAME_UNPROCESSED)
    return _unprocessedFilters.addFilter(p, beforeFilterID);
  else if(frameType == FRAME_RAW_FRAME_PROCESSED)
    return _processedFilters.addFilter(p, beforeFilterID);
  else if(frameType == FRAME_DEPTH_FRAME)
    return _depthFilters.addFilter(p, beforeFilterID);
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Filter not supported for frame type = '" << frameType << "' for camera = " << id() << std::endl;
    return -1;
  }
}

bool DepthCamera::removeAllFilters(FrameType frameType)
{
  if(frameType == FRAME_RAW_FRAME_UNPROCESSED)
    return _unprocessedFilters.removeAllFilters();
  else if(frameType == FRAME_RAW_FRAME_PROCESSED)
    return _processedFilters.removeAllFilters();
  else if(frameType == FRAME_DEPTH_FRAME)
    return _depthFilters.removeAllFilters();
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Filter not supported for frame type = '" << frameType << "' for camera = " << id() << std::endl;
    return false;
  }
}

FilterPtr DepthCamera::getFilter(int filterID, DepthCamera::FrameType frameType) const
{
  if(frameType == FRAME_RAW_FRAME_UNPROCESSED)
    return _unprocessedFilters.getFilter(filterID);
  else if(frameType == FRAME_RAW_FRAME_PROCESSED)
    return _processedFilters.getFilter(filterID);
  else if(frameType == FRAME_DEPTH_FRAME)
    return _depthFilters.getFilter(filterID);
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Filter not supported for frame type = '" << frameType << "' for camera = " << id() << std::endl;
    return nullptr;
  }
}


bool DepthCamera::removeFilter(int filterID, FrameType frameType)
{
  if(frameType == FRAME_RAW_FRAME_UNPROCESSED)
    return _unprocessedFilters.removeFilter(filterID);
  else if(frameType == FRAME_RAW_FRAME_PROCESSED)
    return _processedFilters.removeFilter(filterID);
  else if(frameType == FRAME_DEPTH_FRAME)
    return _depthFilters.removeFilter(filterID);
  else
  {
    logger(LOG_ERROR) << "DepthCamera: Filter not supported for frame type = '" << frameType << "' for camera = " << id() << std::endl;
    return false;
  }
}

bool DepthCamera::refreshParams()
{
  Lock<Mutex> _(_accessMutex);
  bool ret = true;
  for(auto &i: _parameters)
  {
    if(!i.second->refresh())
    {
      logger(LOG_ERROR) << "DepthCamera: Failed to update value for parameter '" << i.first << "'" << std::endl;
      ret = false;
    }
  }
  return ret;
}


void DepthCamera::resetFilters()
{
  _unprocessedFilters.reset();
  _processedFilters.reset();
  _depthFilters.reset();
}

bool DepthCamera::_writeToFrameStream(RawFramePtr &rawUnprocessed)
{
  Lock<Mutex> _(_frameStreamWriterMutex);
  if(_frameStreamWriter && !_frameStreamWriter->write(std::dynamic_pointer_cast<Frame>(rawUnprocessed)))
  {
    logger(LOG_ERROR) << "DepthCamera: Failed to save frames to frame stream" << std::endl;
    return false;
  }
  return true;
}

bool DepthCamera::saveFrameStream(const String &fileName)
{
  Lock<Mutex> _(_frameStreamWriterMutex);
  if(_frameStreamWriter)
  {
    logger(LOG_ERROR) << "DepthCamera: Frame stream is already being saved." << std::endl;
    return false;
  }
  
  if(!_frameGenerators[0] || !_frameGenerators[1] || !_frameGenerators[2])
  {
    logger(LOG_ERROR) << "DepthCamera: Necessary generators are not yet created? Can't save stream." << std::endl;
    return false;
  }
  
  FrameStreamWriterPtr w = FrameStreamWriterPtr(
    new FrameStreamWriter(fileName, _frameGenerators[0]->id(), _frameGenerators[1]->id(), _frameGenerators[2]->id())
  );
  
  _frameGenerators[0]->setFrameStreamWriter(w);
  _frameGenerators[1]->setFrameStreamWriter(w);
  _frameGenerators[2]->setFrameStreamWriter(w);
  
  _frameGenerators[0]->writeConfiguration();
  _frameGenerators[1]->writeConfiguration();
  _frameGenerators[2]->writeConfiguration();
  
  _frameStreamWriter = w;
  
  return _frameStreamWriter->isStreamGood();
}

bool DepthCamera::isSavingFrameStream()
{
  Lock<Mutex> _(_frameStreamWriterMutex);
  return _frameStreamWriter?true:false;
}


bool DepthCamera::closeFrameStream()
{
  Lock<Mutex> _(_frameStreamWriterMutex);
  if(_frameStreamWriter)
  {
    _frameGenerators[0]->removeFrameStreamWriter();
    _frameGenerators[1]->removeFrameStreamWriter();
    _frameGenerators[2]->removeFrameStreamWriter();
    
    _frameStreamWriter->close();
    _frameStreamWriter.reset();
    return true;
  }
  else
  {
    logger(LOG_DEBUG) << "DepthCamera: Frame stream writer not present." << std::endl;
    return false;
  }
}

bool DepthCamera::setCameraProfile(const String &cameraProfileName)
{
  if(!configFile.setCurrentCameraProfile(cameraProfileName))
  {
    logger(LOG_ERROR) << "DepthCamera: Could not set the camera profile to '" << cameraProfileName << "'" << std::endl;
    return false;
  }
  
  /*
  // Uncomment this perform soft-reset every time a profile is selected.
  if(!reset())
  {
    logger(LOG_ERROR) << "DepthCamera: Failed to reset camera, to set new camera profile" << std::endl;
    return false;
  }
  */
  
  if(!refreshParams())
    return false;
  
  ConfigurationFile *config;
  
  if(!(config = configFile.getCameraProfile(cameraProfileName)))
  {
    logger(LOG_ERROR) << "DepthCamera: Failed to get new camera profile information" << std::endl;
    return false;
  }
  
  const ConfigSet *params;
  
  if(config->getConfigSet("params", params) && !_applyConfigParams(params))
  {
    logger(LOG_ERROR) << "DepthCamera: Could not set parameters to initialize profile '" << cameraProfileName << "'" << std::endl;
    return false;
  }
  
  if(config->getConfigSet("defining_params", params) && !_applyConfigParams(params))
  {
    logger(LOG_ERROR) << "DepthCamera: Could not set parameters to initialize profile '" << cameraProfileName << "'" << std::endl;
    return false;
  }
  
  if(!_onReset())
    return false;
  
  return true;
}

bool DepthCamera::_applyConfigParams(const ConfigSet *params)
{
  for(auto i = 0; i < params->paramNames.size(); i++)
  {
    if(params->paramNames[i].compare(0, 2, "0x") == 0)
    {
      logger(LOG_INFO) << "DepthCamera: Setting register '" << params->paramNames[i] << "'" << std::endl;
      
      char *endptr;
      uint32_t reg = (uint32_t)strtol(params->paramNames[i].c_str(), &endptr, 16);
      uint32_t value = (uint32_t)strtol(params->get(params->paramNames[i]).c_str(), &endptr, 0);
      
      if(!_programmer->writeRegister(reg, value))
      {
        logger(LOG_ERROR) << "Failed to write to register @0x" << std::hex << reg << " = 0x" << value << std::dec << std::endl;
      }
      continue;
    }
    else if(params->paramNames[i] == "frame_rate")
    {
      float rate = params->getFloat(params->paramNames[i]);
      FrameRate r;
      r.numerator = rate*10000;
      r.denominator = 10000;
      
      uint g = gcd(r.numerator, r.denominator);
      
      r.numerator /= g;
      r.denominator /= g;
      
      if(!setFrameRate(r)) 
      {
        logger(LOG_ERROR) << "DepthCamera: Failed to set frame rate to " << rate << "fps" << std::endl;
        return false;
      }
      continue;
    }
    
    logger(LOG_INFO) << "DepthCamera: Setting parameter '" << params->paramNames[i] << "'" << std::endl;
    
    const Parameter *p = getParam(params->paramNames[i]).get();
    
    if(!p)
    {
      logger(LOG_ERROR) << "DepthCamera: Ignoring unknown parameter " << params->paramNames[i] << std::endl;
      return false;
    }
    
    const BoolParameter *bp = dynamic_cast<const BoolParameter *>(p);
    const EnumParameter *ep = dynamic_cast<const EnumParameter *>(p);
    const IntegerParameter *ip = dynamic_cast<const IntegerParameter *>(p);
    const UnsignedIntegerParameter *up = dynamic_cast<const UnsignedIntegerParameter *>(p);
    const FloatParameter *fp = dynamic_cast<const FloatParameter *>(p);
    
    if(bp)
    {
      if(!set(params->paramNames[i], params->getBoolean(params->paramNames[i])))
        return false;
    }
    else if(ip || ep)
    {
      if(!set(params->paramNames[i], params->getInteger(params->paramNames[i])))
        return false;
    }
    else if(up)
    {
      if(!set(params->paramNames[i], (uint)params->getInteger(params->paramNames[i])))
        return false;
    }
    else if(fp)
    {
      if(!set(params->paramNames[i], params->getFloat(params->paramNames[i])))
        return false;
    }
    else
    {
      logger(LOG_ERROR) << "DepthCamera: Parameter type unknown for " << params->paramNames[i] << std::endl;
      return false;
    }
  }
  return true;
}

bool DepthCamera::getSerialNumber(String &serialNumber) const 
{ 
  serialNumber = _device->serialNumber(); 
  return true;
}

bool DepthCamera::setSerialNumber(const String &serialNumber) 
{ 
  return false;
}


  
}