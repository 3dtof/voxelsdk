

#include "TemporalMedianFilter.h"

#include <utility>

namespace Voxel
{
  
TemporalMedianFilter::TemporalMedianFilter(uint order, float deadband): Filter("TemporalMedianFilter"), _order(order), _deadband(deadband) 
{
  _addParameters({
    FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", _order, "", 1, 100)),
    FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", _deadband, "", 0, 1)),
  });
}

void TemporalMedianFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "order")
  {
    if(!_get(f->name(), _order))
    {
      logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'order' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadband")
  {
    if(!_get(f->name(), _deadband))
    {
      logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
    }
  }
}

void TemporalMedianFilter::reset() { _history.clear(); _current.clear(); }

template <typename T>
bool TemporalMedianFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  T *cur, *hist;
  
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  cur = (T *)_current.data();
  
  if(_history.size() < _order)
  {
    Vector<ByteType> h;
    h.resize(s*sizeof(T));
    
    memcpy(h.data(), in, s*sizeof(T));
    
    _history.push_back(std::move(h));
    
    memcpy(cur, in, s*sizeof(T));
    memcpy(out, in, s*sizeof(T));
  }
  else
  {
    _history.pop_front();
    
    Vector<ByteType> h;
    h.resize(s*sizeof(T));
    
    memcpy(h.data(), in, s*sizeof(T));
    
    _history.push_back(std::move(h));
    
    for(auto i = 0; i < s; i++)
    {
      T v;
      
      _getMedian(i, v);
      
      if(v > 0 && fabs(((float)v - cur[i])/cur[i]) > _deadband)
        out[i] = cur[i] = v;
      else
        out[i] = cur[i];
    }
  }
  return true;
}

template <typename T>
void TemporalMedianFilter::_getMedian(IndexType offset, T &value)
{
  Vector<T> v;
  
  v.reserve(_order);
  
  for(auto &h: _history)
  {
    T *h1 = (T *)(h.data());
    if(h.size() > offset*sizeof(T))
      v.push_back(h1[offset]);
  }
  
  std::nth_element(v.begin(), v.begin() + v.size()/2,  v.end());
  
  value = v[v.size()/2];
}

bool TemporalMedianFilter::_filter(const FramePtr &in, FramePtr &out)
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
    /*** amplitudeWordWidth and phaseWordWidth are same ***/
    /*** ambientWordWidth and flagsWordWidth are same ***/
   
    unsigned int size1 = s*tofFrame->ambientWordWidth();
    unsigned int size2 = s*tofFrame->amplitudeWordWidth();

    memcpy(o->ambient(), tofFrame->ambient(), size1);
    memcpy(o->amplitude(), tofFrame->amplitude(), size2);
    memcpy(o->flags(), tofFrame->flags(), size1);
    
    if(tofFrame->phaseWordWidth() == 2)
      return _filter<uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      return _filter<uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      return _filter<uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)o->phase());
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
    
    return _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  else
    return false;
}
  
}