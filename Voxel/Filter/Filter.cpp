/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Filter.h"

namespace Voxel
{

bool Filter::_addParameters(const tVector<FilterParameterPtr> &params)
{
  for(auto &p: params)
  {
    auto x = _parameters.find(p->name());
    
    if(x != _parameters.end())
    {
      logger(LOG_ERROR) << "Filter: A parameter with name '" << p->name() << "' already exists" << std::endl;
      return false;
    }
    _parameters[p->name()] = p;
  }
  return true;
}

bool Filter::_prepareOutput(const FramePtr &in, FramePtr &out)
{
  if(out && out->isSameSize(*in))
  {
    out->id = in->id;
    out->timestamp = in->timestamp;
    return true;
  }
  
  out = in->newFrame();
  
  out->id = in->id;
  out->timestamp = in->timestamp;
  
  return true;
}

  
}