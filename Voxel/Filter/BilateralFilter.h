/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_BILATERAL_H
#define VOXEL_BILATERAL_H

#include "Filter.h"

#define _MATH_DEFINES
#include <math.h>

namespace Voxel
{
  
/**
 * \addtogroup Flt
 * @{
 */

template <typename T>
class BilateralFilter: public Filter2D
{
protected:
  float _sigma;
  
  float _fastGaussian(float x2);
  
public:
  BilateralFilter(FrameSize s, float sigma): Filter2D(s), _sigma(sigma)
  {
    _addParameters({
      FilterParameterPtr(new FloatFilterParameter("sigma", "Sigma", "Standard deviation", _sigma, "", 0, 100)),
    });
  }
  
  virtual void reset() {}
  
  virtual const String &name() { return "BilateralFilter"; }
  
  virtual void _onSet(const FilterParameterPtr &f)
  {
    if(f->name() == "sigma")
    {
      if(!get(f->name(), _sigma))
      {
        logger(LOG_WARNING) << "BilateralFilter: Could not get the recently updated 'sigma' parameter" << std::endl;
      }
    }
  }
  
  virtual ~BilateralFilter() {}
  
  template <typename T2>
  bool filter(const Vector<T> &in, const Vector<T2> &ref, Vector<T> &out);
};

template <typename T>
template <typename T2>
bool BilateralFilter<T>::filter(const Vector<T> &in, const Vector<T2> &ref, Vector<T> &out)
{
  uint s = _size.width*_size.height;
  if(in.size() != s || ref.size() != s)
    return false;
  
  if(out.size() != s)
    out.resize(s);
  
  
  for (int j = 0; j < _size.height; j++) 
  {
    for (int i = 0; i < _size.width; i++) 
    {
      int p = j*_size.width+i;
      
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
            int q = j2*_size.width+i2;
            float spatial_dist_squared = k*k + m*m;
            float intensity_dist_squared = (ref[p]-ref[q])*(ref[p]-ref[q]);
            float weight = _fastGaussian(spatial_dist_squared, _sigma)*
            _fastGaussian(intensity_dist_squared, _sigma);
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

template <typename T>
float BilateralFilter<T>::_fastGaussian(float x2)
{
  float fval;
  float sigma2;
  
  if (_sigma == 0) return -1;
  
  sigma2 = _sigma * _sigma;
  fval = x2 / (2.0 * sigma2);
  return exp(-fval)/(2*M_PI*sigma2);
}

/**
 * @}
 */

}

#endif // VOXEL_BILATERAL_H