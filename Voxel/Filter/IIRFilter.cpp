/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "IIRFilter.h"

namespace Voxel
{
  
IIRFilter::IIRFilter(float gain): Filter("IIRFilter"), _gain(gain)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("gain", "Gain", "IIR gain coefficient", gain, "", 0.0f, 1.0f))
  });
}

void IIRFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "gain")
    if(!get(f->name(), _gain))
    {
      logger(LOG_WARNING) << "IIRFilter:  Could not get the recently updated 'gain' parameter" << std::endl;
    }
}

void IIRFilter::reset()
{
  _current.clear();
}

template <typename T>
bool IIRFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  T *cur;
  
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  cur = (T *)_current.data();
  
  for(auto i = 0; i < s; i++) 
    out[i] = cur[i] = cur[i]*(1.0 - _gain) + in[i]*_gain;
  
  return true;
}

bool IIRFilter::filter(const FramePtr &in, FramePtr &out)
{
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o;
    
    if(!out)
      out = tofFrame->newFrame();
    
    out->id = in->id;
    out->timestamp = in->timestamp;
    
    o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    if(o->size != _size || o->phaseWordWidth() != tofFrame->phaseWordWidth()
      || o->ambientWordWidth() != tofFrame->ambientWordWidth()
      || o->amplitudeWordWidth() != tofFrame->amplitudeWordWidth()
      || o->flagsWordWidth() != tofFrame->flagsWordWidth())
    {
      logger(LOG_ERROR) << "IIRFilter: Output frame is not of appropriate size" << std::endl;
      return false;
    }
    
    //logger(LOG_INFO) << "IIRFilter: Applying filter with gain = " << _gain << " to ToFRawFrame id = " << tofFrame->id << std::endl;
    
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());
    
    if(tofFrame->phaseWordWidth() == 2)
      return _filter<uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      return _filter<uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      return _filter<uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)o->phase());
    else
      return false;
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    
    if(!out)
      out = depthFrame->newFrame();
    
    out->id = in->id;
    out->timestamp = in->timestamp;
    
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    if(o->size != _size)
    {
      logger(LOG_ERROR) << "IIRFilter: Output frame is not of appropriate size" << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    return _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  else
    return false;
}


  
}