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
  
/**
 * \addtogroup Flt
 * @{
 */

class FilterDescription
{
  typedef Function<FilterPtr()> _CreateFilter;
protected:
  _CreateFilter _createFilter;
public:
  String name;
  uint frameTypes; // supported frame types (see #DepthCamera::FrameType)
  
  FilterDescription() {}
  
#ifndef SWIG
  FilterDescription(const String &n, uint f, _CreateFilter c): name(n), frameTypes(f), _createFilter(c) {}
#endif
  
  inline bool supports(DepthCamera::FrameType type)
  {
    return frameTypes & (1 << type);
  }
  
  friend class FilterFactory;
};

class VOXEL_EXPORT FilterFactory
{
  
protected:
  Map<String, FilterDescription> _supportedFilters;
  bool _addSupportedFilters(const Vector<FilterDescription> &f);
  
  String _name;
  
public:
  FilterFactory(const String &name): _name(name) {}
  
  inline const String &name() const { return _name; }
  
  inline const Map<String, FilterDescription> &getSupportedFilters() const { return _supportedFilters; }
  
  virtual FilterPtr createFilter(const String &name, DepthCamera::FrameType type);
};

typedef Ptr<FilterFactory> FilterFactoryPtr;

#ifndef SWIG
// Implement this function in support voxel library
extern "C" void getFilterFactory(FilterFactoryPtr &filterFactory);
#endif

typedef void (*GetFilterFactory)(FilterFactoryPtr &filterFactory); // Function type to return DepthCameraFactory

/**
 * @}
 */

}


#endif
