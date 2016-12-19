/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "MedianFilter.h"

#include <float.h>

namespace Voxel
{

MedianFilter::MedianFilter(float stability, float deadband, float deadbandStep, uint halfKernelSize): Filter("MedianFilter"), 
  _stability(stability), _deadband(deadband), _deadbandStep(deadbandStep), _halfKernelSize(halfKernelSize)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("stability", "Stability", "Stability factor", stability, "", 0, 1)),
    FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", deadband, "", 0, 1)),
    FilterParameterPtr(new FloatFilterParameter("deadbandStep", "Dead band step", "Dead band step", deadbandStep, "", 0, 1)),
    FilterParameterPtr(new UnsignedFilterParameter("halfKernelSize", "Half kernel size", "Half kernel size", halfKernelSize, "", 1, 100))
  });
}

void MedianFilter::reset() { _current.clear(); }

void MedianFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "stability")
  {
    if(!_get(f->name(), _stability))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'stability' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadband")
  {
    if(!_get(f->name(), _deadband))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadbandStep")
  {
    if(!_get(f->name(), _deadbandStep))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadbandStep' parameter" << std::endl;
    }
  }
  else if(f->name() == "halfKernelSize")
  {
    if(!_get(f->name(), _halfKernelSize))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'halfKernelSize' parameter" << std::endl;
    }
  }
}
 
template <typename T>
bool MedianFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  T *cur, *hist;
  
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  uint histSize = (2*_halfKernelSize + 1)*(2*_halfKernelSize + 1);
  
  if(_hist.size() != histSize*sizeof(T))
  {
    _hist.resize(histSize*sizeof(T));
    memset(_hist.data(), 0, histSize*sizeof(T));
  }
  
  cur = (T *)_current.data();
  hist = (T *)_hist.data();
  
  int stablePixel = _size.width*_size.height;
  
  int index;
  
  for (int j = 0; j < _size.height; j++) 
  {
    for (int i = 0; i < _size.width; i++) 
    {
      int p = j*_size.width + i;
      index = 0;
      
      for (int k = -(int)_halfKernelSize; k <= (int)_halfKernelSize; k++) 
      {
        for (int m = -(int)_halfKernelSize; m <= (int)_halfKernelSize; m++) 
        {
          int i2 = i+m; int j2 = j+k;
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
          {
            int q = j2*_size.width+i2;
            hist[index++] = in[q];
          }
        }
      }
      std::nth_element(hist, hist + index/2, hist + index);
      T val = hist[index/2];
      
      // Output w/ deadband
      float ferr = cur[p]?fabs((float)(val- cur[p])/cur[p]):FLT_MAX;
      
      if (ferr > _deadband) 
      {
        out[p] = cur[p] = val;
        stablePixel--;
      }
      else
        out[p] = cur[p];
    }  // for (i)
  } // for (j)
  
  // Adjust deadband until ratio is achieved
  float diff = (float)stablePixel - _stability*_size.width*_size.height;
  if (diff < 0)
    _deadband += _deadbandStep;
  else
    _deadband -= _deadbandStep;
  
  _set("deadband", _deadband);
  
  return true;
}


bool MedianFilter::_filter(const FramePtr &in, FramePtr &out)
{
  bool ret;
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "IIRFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    //logger(LOG_INFO) << "IIRFilter: Applying filter with gain = " << _gain << " to ToFRawFrame id = " << tofFrame->id << std::endl;
    
    uint s = _size.width*_size.height;
    /*** amplitudeWordWidth and phaseWordWidth are same ***/
    /*** ambientWordWidth and flagsWordWidth are same ***/
    
    unsigned int size1 = s*tofFrame->ambientWordWidth();
    unsigned int size2 = s*tofFrame->amplitudeWordWidth();

    memcpy(o->ambient(), tofFrame->ambient(), size1);
    memcpy(o->amplitude(), tofFrame->amplitude(), size2);
    memcpy(o->flags(), tofFrame->flags(), size1);
    
    if(tofFrame->phaseWordWidth() == 2)
      ret = _filter<uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      ret = _filter<uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      ret = _filter<uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)o->phase());
    else
      return false;
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    ret = _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  return ret;
}
  
}