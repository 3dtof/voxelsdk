/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FILTER_FACTORY_H
#define VOXEL_FILTER_FACTORY_H

#include <Common.h>
#include <DepthCamera.h>

namespace Voxel
{

class FilterFactory
{
public:
  struct FilterDescription
  {
    String name;
    uint frameTypes; // supported frame types (see #DepthCamera::FrameType)
    
    inline bool supports(DepthCamera::FrameType type)
    {
      return frameTypes & (1 << type);
    }
  };
  
protected:
  Vector<FilterDescription> _supportedFilters;
  
  inline bool _addSupportedFilters(const Vector<FilterDescription> &f);
  
  String _name;
  
public:
  FilterFactory(const String &name): _name(name) {}
  
  inline const String &name() const { return _name; }
  
  inline const Vector<FilterDescription> &getSupportedFilters() const { return _supportedFilters; }
  
  virtual FilterPtr createFilter(const String &name, DepthCamera::FrameType type) = 0;
};

bool FilterFactory::_addSupportedFilters(const Vector<FilterDescription> &f)
{
  _supportedFilters.reserve(_supportedFilters.size() + f.size());
  _supportedFilters.insert(_supportedFilters.end(), f.begin(), f.end());
}

typedef Ptr<FilterFactory> FilterFactoryPtr;

// Implement this function in support voxel library
extern "C" void getFilterFactory(FilterFactoryPtr &filterFactory);
typedef void (*GetFilterFactory)(FilterFactoryPtr &filterFactory); // Function type to return DepthCameraFactory
  
}


#endif
