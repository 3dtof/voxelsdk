/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "FilterFactory.h"

namespace Voxel
{
  
FilterPtr FilterFactory::createFilter(const String &name, DepthCamera::FrameType type)
{
  auto x = _supportedFilters.find(name);
  
  if(x == _supportedFilters.end())
  {
    logger(LOG_ERROR) << "FilterFactory: Could not find filter '" << name << "'" << std::endl;
    return nullptr;
  }
  
  if(!x->second.supports(type))
  {
    logger(LOG_ERROR) << "FilterFactory: Filter '" << name << "' does not support frame type '" << type << "'" << std::endl;
    return nullptr;
  }
  
  if(!x->second._createFilter)
  {
    logger(LOG_ERROR) << "FilterFactory: Don't know how to create filter '" << name << "'. Report to vendor of FilterFactory '" << _name << "'" << std::endl;
    return nullptr;
  }
  
  FilterPtr p = x->second._createFilter();
  
  if(p) p->setNameScope(_name);
  
  return p;
}

bool FilterFactory::_addSupportedFilters(const Vector<FilterDescription> &f)
{
  _supportedFilters.reserve(_supportedFilters.size() + f.size());
  
  for(auto &x: f)
  {
    String n = _name + "::" + x.name;
    
    auto y = _supportedFilters.find(n);
    
    if(y != _supportedFilters.end())
    {
      logger(LOG_ERROR) << "FilterFactory: A filter by name '" << n << "' already exists. Skipping the new one." << std::endl;
      continue;
    }
    _supportedFilters[n] = x;
  }
  return true;
}

  
}