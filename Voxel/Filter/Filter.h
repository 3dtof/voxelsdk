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

/**
 * \defgroup Flt Filter related classes
 * @{
 */
  
class VOXEL_EXPORT Filter
{
protected:
  String _name, _nameScope;
  String _id;
  
  inline void _makeID() 
  {
    if(_nameScope.size()) 
      _id = _nameScope + "::" + _name; 
    else 
      _id = _name;
  }
  
  DepthCameraPtr _depthCamera;
  
  Map<String, FilterParameterPtr> _parameters;
  
  virtual bool _addParameters(const Vector<FilterParameterPtr> &params); 
  
  // On setting of a parameter this is called
  virtual void _onSet(const FilterParameterPtr &f) = 0;
  
public:
  Filter(const String &name, const String &nameScope = ""): _name(name), _nameScope(nameScope) 
  {
    _makeID();
  }
  
  inline void setDepthCamera(const DepthCameraPtr &d)
  {
    _depthCamera = d;
  }
  
  inline const Map<String, FilterParameterPtr> &parameters() { return _parameters; }

  inline const String &name() { return _id; }
  
  inline void setNameScope(const String &scope) { _nameScope = scope; _makeID(); }
  
  virtual bool filter(const FramePtr &in, FramePtr &out) = 0;
  virtual void reset() = 0;
  
  inline FilterParameterPtr getParam(const String &name) const;
  
  template <typename T>
  bool get(const String &name, T &value);
  
  template <typename T>
  bool set(const String &name, const T &value);
  
  virtual ~Filter() {}
};

FilterParameterPtr Filter::getParam(const String &name) const
{
  auto p = _parameters.find(name);
  
  if(p != _parameters.end())
    return p->second;
  else
    return nullptr;
}


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

/**
 * @}
 */


}


#endif //VOXEL_FILTER_H
