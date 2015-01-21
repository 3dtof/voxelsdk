/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FILTER_H
#define VOXEL_FILTER_H


#include <Frame.h>
#include <FrameBuffer.h>
#include <Filter/FilterParameter.h>
#include <Logger.h>

#include <memory>

#include "VoxelExports.h"

namespace Voxel
{
  
class DepthCamera;

typedef Ptr<DepthCamera> DepthCameraPtr;
  
class VOXEL_EXPORT Filter
{
  DepthCameraPtr _depthCamera;
  
  Map<String, FilterParameterPtr> _parameters;
  
  // On setting of a parameter this is called
  virtual void _onSet(const FilterParameterPtr &f) = 0;
  
  virtual bool _addParameters(const Vector<FilterParameterPtr> &params); 
  
public:
  inline void setDepthCamera(const DepthCameraPtr &d)
  {
    _depthCamera = d;
  }
  
  inline const Map<String, FilterParameterPtr> &parameters() { return _parameters; }

  virtual const String &name() = 0;
  virtual void reset() = 0;
  
  template <typename T>
  bool get(const String &name, T &value);
  
  template <typename T>
  bool set(const String &name, const T &value);
  
  virtual ~Filter() {}
};

template <typename T>
bool Filter::get(const String &name, T &value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    FilterParameterTemplate<T> *f = dynamic_cast<FilterParameterTemplate<T> *>(p->second.get());
    
    if(!f)
    {
      logger(LOG_ERROR) << "Filter: Invalid type '" << typeid(value).name() << "' to get parameter '" << name << "'" << std::endl;
      return false;
    }
    
    return f->get(value);
  }
  else
  {
    logger(LOG_ERROR) << "Filter: Unknown parameter name '" << name << "'" << std::endl;
    return false;
  }
}

template <typename T>
bool Filter::set(const String &name, const T &value)
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
  {
    FilterParameterTemplate<T> *f = dynamic_cast<FilterParameterTemplate<T> *>(p->second.get());
    
    if(!f)
    {
      logger(LOG_ERROR) << "Filter: Invalid type '" << typeid(value).name() << "' to get parameter '" << name << "'" << std::endl;
      return false;
    }
    
    if(f->set(value))
    {
      _onSet(p->second);
      return true;
    }
    else
      return false;
  }
  else
  {
    logger(LOG_ERROR) << "Filter: Unknown parameter name '" << name << "'" << std::endl;
    return false;
  }
}

typedef Ptr<Filter> FilterPtr;


class VOXEL_EXPORT Filter2D: public Filter
{
protected:
  FrameSize _size;
public:
  Filter2D() { _size.width = _size.height = 0; }
  Filter2D(FrameSize s): _size(s) {}
  
  inline const FrameSize &size() const { return _size; }
  
  inline void setSize(FrameSize s) { _size = s; reset(); }
  
  virtual ~Filter2D() {}
};

class VOXEL_EXPORT FrameFilter: public Filter
{
public:
  virtual bool filter(const FramePtr &in, FramePtr &out) = 0;
  
  virtual ~FrameFilter() {}
};

typedef Ptr<FrameFilter> FrameFilterPtr;

}


#endif //VOXEL_FILTER_H
