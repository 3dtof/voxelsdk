/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_MEDIAN_FILTER_H
#define VOXEL_MEDIAN_FILTER_H

#include <Filter/Filter.h>
#include <memory.h>

namespace Voxel
{

template <typename T>
class MedianFilter: public Filter2D
{
protected:
  float _stability, _deadband, _deadbandStep; 
  uint _halfKernelSize;
  
  Vector<T> _current;
  
public:
  MedianFilter(FrameSize s, float stability, float deadband, float deadbandStep, uint halfKernelSize):
  Filter2D(s), _stability(stability), _deadband(deadband), _deadbandStep(deadbandStep), _halfKernelSize(halfKernelSize)
  {
    _addParameters({
      FilterParameterPtr(new FloatFilterParameter("stability", "Stability", "Stability factor", stability, "", 0, 1)),
      FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", deadband, "", 0, 1)),
      FilterParameterPtr(new FloatFilterParameter("deadbandStep", "Dead band step", "Dead band step", deadbandStep, "", 0, 1)),
      FilterParameterPtr(new UnsignedFilterParameter("halfKernelSize", "Half kernel size", "Half kernel size", halfKernelSize, "", 1, 100))
    });
  }
  virtual ~MedianFilter() {}
  
  virtual const String &name() { return "MedianFilter"; }
  virtual void reset() { _current.reset(); }
  
  virtual void _onSet(const FilterParameterPtr &f)
  {
    if(f->name() == "stability")
    {
      if(!get(f->name(), _stability))
      {
        logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'stability' parameter" << std::endl;
      }
    }
    else if(f->name() == "deadband")
    {
      if(!get(f->name(), _deadband))
      {
        logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
      }
    }
    else if(f->name() == "deadbandStep")
    {
      if(!get(f->name(), _deadbandStep))
      {
        logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadbandStep' parameter" << std::endl;
      }
    }
    else if(f->name() == "halfKernelSize")
    {
      if(!get(f->name(), _halfKernelSize))
      {
        logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'halfKernelSize' parameter" << std::endl;
      }
    }
  }
  
  bool filter(const Vector<T> &in, Vector<T> &out);
};

template <typename T>
bool MedianFilter::filter(const Vector<T> &in, Vector<T> &out)
{
  uint s = _size.width*_size.height;
  if(in.size() != s)
    return false;
  
  if(out.size() != s)
    out.resize(s);
  
  if(_current.size() != s)
  {
    _current.resize(s);
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  Vector<T> hist;
  
  hist.reserve((2*halfKernelSize + 1)*(2*halfKernelSize + 1));
  
  int stablePixel = _size.width*_size.height;
  
  for (int j = 0; j < _size.height; j++) 
  {
    for (int i = 0; i < _size.width; i++) 
    {
      int p = j*_size.width + i;
      hist.clear();
      
      for (int k = -_halfKernelSize; k <= _halfKernelSize; k++) 
      {
        for (int m = -_halfKernelSize; m <= _halfKernelSize; m++) 
        {
          int i2 = i+m; int j2 = j+k;
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
          {
            int q = j2*_size.width+i2;
            hist.push_back(in[q]);
          }
        }
      }
      std::nth_element(hist.begin(), hist.begin() + hist.size()/2, hist.end());
      T val = hist[hist.size()/2];
      
      // Output w/ deadband
      float ferr = fabs((float)(val- _current[p])/_current[p]);
      
      if (ferr > _deadband) 
      {
        out[p] = _current[p] = val;
        stablePixel--;
      }
      else
        out[p] = _current[p];
    }  // for (i)
  } // for (j)
  
  // Adjust deadband until ratio is achieved
  float diff = (float)stablePixel - _stability*_size.width*_size.height;
  if (diff < 0)
    _deadband += _deadbandStep;
  else
    _deadband -= _deadbandStep;
  
  set("deadband", _deadband);
  
  return true;
}


}
#endif // VOXEL_MEDIAN_FILTER_H