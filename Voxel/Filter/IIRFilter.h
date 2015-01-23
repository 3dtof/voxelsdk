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
  
class VOXEL_EXPORT IIRFilter: public Filter
{
protected:
  float _gain;
  Vector<ByteType> _current;
  
  FrameSize _size;
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual void _onSet(const FilterParameterPtr &f);
  
public:
  IIRFilter(float gain = 0.5);
  
  virtual void reset();
  
  virtual bool filter(const FramePtr &in, FramePtr &out);
  
  virtual ~IIRFilter() {}
};

}
#endif // VOXEL_IIR_H
