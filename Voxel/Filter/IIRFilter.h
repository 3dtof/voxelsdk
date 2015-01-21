/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_IIR_H
#define VOXEL_IIR_H

#include "Filter.h"

#include <string.h>

namespace Voxel
{

template <typename T>
class IIRFilter: public Filter2D
{
protected:
  float _gain;
  Vector<T> _current;
  
public:
  IIRFilter(FrameSize s, float gain): _gain(gain), Filter2D(s) 
  {
    _addParameters({
     FilterParameterPtr(new FloatFilterParameter("gain", "Gain", "IIR gain coefficient", "", 0.1f, 0.99f, gain))
    });
  }
  
  virtual const String &name() { return "IIRFilter"; }
  
  virtual void _onSet(const FilterParameterPtr &f)
  {
    if(f->name() == "gain")
      if(!get(f->name(), _gain))
      {
        logger(LOG_WARNING) << "IIRFilter: Could not get the recently updated 'gain' parameter" << std::endl;
      }
  }
  
  virtual void reset() { _current.clear(); }
  
  bool filter(const Vector<T> &in, Vector<T> &out);
  
  virtual ~IIRFilter() {}
};

template <typename T>
bool IIRFilter<T>::filter(const Vector<T> &in, Vector<T> &out)
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
  
  for(auto i = 0; i < s; i++) 
    out[i] = _current[i] = _current[i]*(1.0 - _gain) + in[i]*_gain;
  
  return true;
}

}
#endif // VOXEL_IIR_H
