/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DarkPixFilter.h"

namespace Voxel
{
  
DarkPixFilter::DarkPixFilter(float aThrNear, float phThrNear, float aThrFar, float phThrFar): Filter("DarkPixFilter"), _aThrNear(aThrNear), _phThrNear(phThrNear), _aThrFar(aThrFar), _phThrFar(phThrFar)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("aThrNear", "aThrNear", "NearAmpThr", _aThrNear, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("phThrNear", "phThrNear", "NearPhThr", _phThrNear, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("aThrFar", "aThrFar", "FarAmpThr", _aThrFar, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("phThrFar", "phThrFar", "FarPhThr", _phThrFar, "", 0.0f, 4095.0f))
  });
}

void DarkPixFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "aThrNear")
  {
    if(!_get(f->name(), _aThrNear))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'aThrNear' parameter" << std::endl;
    }
  }
  else if(f->name() == "phThrNear") 
  {
    if(!_get(f->name(), _phThrNear))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'phThrNear' parameter" << std::endl;
    }
  }
  else if(f->name() == "aThrFar")
  {
    if(!_get(f->name(), _aThrFar))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'aThrFar' parameter" << std::endl;
    }
  }
  else if(f->name() == "phThrFar") 
  {
    if(!_get(f->name(), _phThrFar))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'phThrFar' parameter" << std::endl;
    }
  }
}

void DarkPixFilter::reset()
{
  _current.clear();
}

template <typename T>
bool DarkPixFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  for(auto i = 0; i < s; i++) 
    out[i] = in[i];
  
  return true;
}

template <typename T, typename T2>
bool DarkPixFilter::_filter2(const T *in, T2 *amp, T *out)
{
    uint s = _size.width*_size.height;

    for (auto p = 0; p < s; p++)
    {
      if ( ((amp[p] < _aThrNear) && (in[p] < _phThrNear)) ||  ((amp[p] > _aThrFar) && (in[p] > _phThrFar))  )
         out[p] = 0;
      else
         out[p] = in[p];
    }
    return true;
}

bool DarkPixFilter::_filter(const FramePtr &in, FramePtr &out)
{
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "DarkPixFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "DarkPixFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    //logger(LOG_INFO) << "DarkPixFilter: Applying filter with aThr = " << _aThr << " and phThr = " << _phThr << std::endl;
    
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    //memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());

    if(tofFrame->phaseWordWidth() == 2)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter2<uint16_t, uint8_t>((uint16_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2) {
        _filter2<uint16_t, uint16_t>((uint16_t *)tofFrame->amplitude(), (uint16_t *)tofFrame->amplitude(), (uint16_t *)o->amplitude());
        return _filter2<uint16_t, uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
      }
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter2<uint16_t, uint32_t>((uint16_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 1)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter2<uint8_t, uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2)
        return _filter2<uint8_t, uint16_t>((uint8_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter2<uint8_t, uint32_t>((uint8_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 4)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter2<uint32_t, uint8_t>((uint32_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2) {
        return _filter2<uint32_t, uint16_t>((uint32_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
      }
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter2<uint32_t, uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
    }
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "DarkPixFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    return _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  else
    return false;
}
  
}
