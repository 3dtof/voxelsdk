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
  
/**
 * \addtogroup Flt
 * @{
 */

class MedianFilter: public Filter
{
protected:
  float _stability, _deadband, _deadbandStep; 
  uint _halfKernelSize;
  
  Vector<ByteType> _current, _hist;
  
  FrameSize _size;
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  MedianFilter(float stability = 0.1, float deadband = 0.05, float deadbandStep = 0.01, uint halfKernelSize = 1);
  virtual ~MedianFilter() {}
  
  virtual void reset();
};

/**
 * @}
 */


}
#endif // VOXEL_MEDIAN_FILTER_H