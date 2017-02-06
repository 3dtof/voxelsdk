
#include "SmoothFilter.h"

#include <memory.h>

namespace Voxel
{

SmoothFilter::SmoothFilter(float sigma): Filter("SmoothFilter"), _discreteGaussian(sigma)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("sigma", "Sigma", "Standard deviation", sigma, "", 0, 100))
  });
}

void SmoothFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "sigma")
  {
    float s;
    if(!_get(f->name(), s))
    {
      logger(LOG_WARNING) << "SmoothFilter: Could not get the recently updated 'sigma' parameter" << std::endl;
    }
    _discreteGaussian.setStandardDeviation(s);
  }
}

void SmoothFilter::reset() {}

template <typename T>
bool SmoothFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  for (int j = 0; j < _size.height; j++) {
    for (int i = 0; i < _size.width; i++) {
      int p = j*_size.width + i;
      float weight_sum = 0;
      float sum = 0;
      for (int k = -2; k <= 2; k++) {
        for (int m = -2; m <= 2; m++) {
          int i2 = i + m;
          int j2 = j + k;
          
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) {
            int q = j2*_size.width + i2;
            float weight = _discreteGaussian.valueAt(k)*_discreteGaussian.valueAt(m);
            weight_sum += weight;
            sum += weight * in[q];
          }
        }
      }
      out[p] = ((T)(sum / weight_sum));
    }
  }
  return true;  
}

bool SmoothFilter::_filter(const FramePtr &in, FramePtr &out)
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
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());
    
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