/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TEMPORAL_MEDIAN_FILTER_H
#define VOXEL_TEMPORAL_MEDIAN_FILTER_H

#include "Filter.h"

#include <string.h>

namespace Voxel
{
/**
 * \addtogroup Flt
 * @{
 */
template <typename T>
class TemporalMedianFilter: public Filter2D
{
protected:
  float _deadband;
  uint _order;
  
  List<Vector<T>> _history;
  
  Vector<T> _current;
  
  T _getMedian(IndexType offset);
  
public:
  TemporalMedianFilter(FrameSize s, uint order, float deadband): Filter2D(s), _order(order), _deadband(deadband) 
  {
    _addParameters({
      FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", "", 1, 100, _order)),
      FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", "", 0, 1, _deadband)),
    });
  }
  
  virtual ~TemporalMedianFilter() {}
  
  virtual const String &name() { return "TemporalMedianFilter"; }
  
  virtual void _onSet(const FilterParameterPtr &f)
  {
    if(f->name() == "order")
    {
      if(!get(f->name(), _order))
      {
        logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'order' parameter" << std::endl;
      }
    }
    else if(f->name() == "deadband")
    {
      if(!get(f->name(), _deadband))
      {
        logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
      }
    }
  }
  
  virtual void reset() { _history.clear(); _current.clear(); }
  
  bool filter(const Vector<T> &in, Vector<T> &out);
};

template <typename T>
bool TemporalMedianFilter<T>::filter(const Vector<T> &in, Vector<T> &out)
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
  
  if(_history.size() < _order)
  {
    _history.push_back(in);
    out = _current = in;
  }
  else
  {
    _history.pop_front();
    _history.push_back(in);
    
    for(auto i = 0; i < s; i++)
    {
      T v = _getMedian(i);
      
      if(v > 0 && fabs(((float)v - _current[i])/_current[i]) > _deadband)
        out[i] = _current[i] = v;
      else
        out[i] = _current[i];
    }
  }
  return true;
}

template <typename T>
T TemporalMedianFilter<T>::_getMedian(IndexType offset)
{
  Vector<T> v;
  
  v.reserve(_order);
  
  for(auto &h: _history)
  {
    if((*h).size() > offset)
      v.push_back((*h)[offset]);
  }
  
  std::nth_element(v.begin(), v.begin() + v.size()/2,  v.end());
  
  return v[v.size()/2];
}
/**
 * @}
 */

}
#endif // VOXEL_TEMPORAL_MEDIAN_FILTER_H
