/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DISCRETE_GUASSIAN_H
#define VOXEL_DISCRETE_GUASSIAN_H

#include "Common.h"

#define _MATH_DEFINES
#include <math.h>


namespace Voxel
{
  
class VOXEL_EXPORT DiscreteGaussian
{
  float _sigma, _squaredSigma;
  
  Vector<float> _values;
  
  inline void _computeValues();
  
public:
  DiscreteGaussian(float s) { setStandardDeviation(s); }
  
  inline void setStandardDeviation(float s)
  {
    _sigma = s;
    _squaredSigma = _sigma*_sigma;
    _computeValues();
  }
  
  inline float getStandardDeviation() const { return _sigma; }
  inline float getSquaredStandardDeviation() const { return _squaredSigma; }
  
  inline float valueAt(int x)
  {
    int y = (x < 0)?-x:x;
    
    if(y >= _values.size())
      return 0.0f;
    else
      return _values[y];
  }
};

inline void DiscreteGaussian::_computeValues()
{
  int limit = sqrt(20)*_sigma + 1; // This would correspond to exp(-10)
  
  _values.resize(limit);
  
  if(!floatEquals(_sigma, 0))
  {
    float d = _sigma*sqrt(2*M_PI);
    
    for(auto i = 0; i < limit; i++)
      _values[i] = exp(-i*i/2.0f/_squaredSigma)/d;
  }
  else
    _values[0] = 1;
}

  
}

#endif
