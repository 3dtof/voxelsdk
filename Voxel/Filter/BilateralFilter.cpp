
#include "BilateralFilter.h"

#include <memory.h>

namespace Voxel
{
  
BilateralFilter::BilateralFilter(float sigma): Filter("BilateralFilter"), _sigma(sigma)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("sigma", "Sigma", "Standard deviation", _sigma, "", 0, 100)),
  });
}

void BilateralFilter::reset() {}

void BilateralFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "sigma")
  {
    if(!_get(f->name(), _sigma))
    {
      logger(LOG_WARNING) << "BilateralFilter: Could not get the recently updated 'sigma' parameter" << std::endl;
    }
  }
}

template <typename T, typename T2>
bool BilateralFilter::_filter(const  T *in, const T2 *ref, T *out)
{
  uint s = _size.width*_size.height;
  
  #pragma omp parallel for
  for (int j = 0; j < _size.height; j++) 
  {
    for (int i = 0; i < _size.width; i++) 
    {
      int p = j*_size.width + i;
      
      float weight_sum = 0;
      float sum = 0;
      
      for (int k = -2; k <= 2; k++) 
      {
        for (int m = -2; m <= 2; m++) 
        {                         
          int i2 = i+m;
          int j2 = j+k;
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
          {
            int q = j2*_size.width + i2;
            float spatial_dist_squared = k*k + m*m;
            float intensity_dist_squared = (ref[p]-ref[q])*(ref[p]-ref[q]);
            float weight = _fastGaussian(spatial_dist_squared)*
            _fastGaussian(intensity_dist_squared);
            weight_sum += weight;
            sum += weight * in[q];
          }
        }
      }
      out[p] = (T)(sum / weight_sum);
    }
  }
  return true;
}

float BilateralFilter::_fastGaussian(float x2)
{
  float fval;
  float sigma2;
  
  if (_sigma == 0) return -1;
  
  sigma2 = _sigma * _sigma;
  fval = x2 / (2.0 * sigma2);
  return exp(-fval)/(2*M_PI*sigma2);
}

bool BilateralFilter::_filter(const FramePtr &in, FramePtr &out)
{
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
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter<uint16_t, uint8_t>((uint16_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2)
        return _filter<uint16_t, uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter<uint16_t, uint32_t>((uint16_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint16_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 1)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter<uint8_t, uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2)
        return _filter<uint8_t, uint16_t>((uint8_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter<uint8_t, uint32_t>((uint8_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint8_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 4)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        return _filter<uint32_t, uint8_t>((uint32_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2)
        return _filter<uint32_t, uint16_t>((uint32_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 4)
        return _filter<uint32_t, uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), (uint32_t *)o->phase());
    }
     
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
    
    return _filter<float, float>(depthFrame->depth.data(), depthFrame->amplitude.data(), o->depth.data());
  }
  else
    return false;
}
  
}