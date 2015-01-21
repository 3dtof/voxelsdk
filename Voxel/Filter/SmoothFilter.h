/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_SMOOTH_H
#define VOXEL_SMOOTH_H

#include "Filter.h"

#define _MATH_DEFINES
#include <math.h>

namespace Voxel
{

template <typename T>
class SmoothFilter: public Filter2D 
{
protected:
  float _sigma;
  
  float _fastGaussian(float x2);
  
public:
  SmoothFilter(FrameSize s, float sigma): Filter2D(s), _sigma(sigma) 
  {
    _addParameters({
      FilterParameterPtr(new FloatFilterParameter("sigma", "Sigma", "Standard deviation", "", 0, 100, _sigma))
    });
  }
  virtual ~SmoothFilter() {}
  
  virtual const String &name() { return "SmoothFilter"; }
  
  virtual void _onSet(const FilterParameterPtr &f)
  {
    if(f->name() == "sigma")
      if(!get(f->name(), _sigma))
      {
        logger(LOG_WARNING) << "SmoothFilter: Could not get the recently updated 'sigma' parameter" << std::endl;
      }
  }
  
  virtual void reset() {}
  
  bool filter(const Vector<T> &in, Vector<T> &out);
};

template <typename T>
bool SmoothFilter::filter(const Vector<T>& in, Vector<T>& out)
{
  uint s = _size.width*_size.height;
  if(in.size() != s)
    return false;
  
  if(out.size() != s)
    out.resize(s);
  
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
            float spatial_dist_squared = k*k + m*m;
            float weight = _fastGaussian(spatial_dist_squared);
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

template <typename T>
float SmoothFilter<T>::_fastGaussian(float x2)
{
  float fval;
  float sigma2;
  
  if (_sigma == 0) return -1;
  
  sigma2 = _sigma * _sigma;
  fval = x2 / (2.0 * sigma2);
  return exp(-fval)/(2*M_PI*sigma2);
}


}
#endif // VOXEL_SMOOTH_H
