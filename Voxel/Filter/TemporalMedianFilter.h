/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TEMPORAL_MEDIAN_FILTER_H
#define VOXEL_TEMPORAL_MEDIAN_FILTER_H

#include "Filter.h"
#ifdef ARM_OPT
#include <arm_neon.h>
extern int32_t nFrameWidth, nFrameHeight;
#endif

#include <string.h>

namespace Voxel
{
/**
 * \addtogroup Flt
 * @{
 */
#ifdef ARM_OPT
#define vmax_min(a,b) \
                      { uint16x8_t vtemp = a;\
                       a = vmaxq_u16(a, b);\
                       b = vminq_u16(vtemp, b);} 
#endif
class TemporalMedianFilter: public Filter
{
protected:
  float _deadband;
  uint _order;
  
  FrameSize _size;
#ifndef ARM_OPT
  List<Vector<ByteType>> _history;
  Vector<ByteType> _current;
  
#else
  ByteType **_history;
  ByteType *_current;
  int number_frames = 0;
  int frame_cnt = 0;
#endif
  virtual void _onSet(const FilterParameterPtr &f);
  
  template <typename T>
  void _getMedian(IndexType offset, T &value);
  
  template <typename T>
  bool _filter(const T *in, T *out);
  
  virtual bool _filter(const FramePtr &in, FramePtr &out);
  
public:
  TemporalMedianFilter(uint order = 3, float deadband = 0.05);
  virtual ~TemporalMedianFilter() {}
  
  virtual void reset();
};

/**
 * @}
 */

}
#endif // VOXEL_TEMPORAL_MEDIAN_FILTER_H
