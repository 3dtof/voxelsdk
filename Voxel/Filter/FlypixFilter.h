/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#ifndef VOXEL_FLYPIX_H
#define VOXEL_FLYPIX_H

#include "Filter.h"

#define _MATH_DEFINES
#include <math.h>

namespace Voxel
{
  
/**
 * \addtogroup Flt
 * @{
 */

class FlypixFilter: public Filter 
{
protected:
  float _thr;
  
  FrameSize _size;
  
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  FlypixFilter(float thr = 500);
  virtual ~FlypixFilter() {}
  
  virtual void reset();
};


/**
 * @}
 */

}
#endif // VOXEL_FLYPIX_H
