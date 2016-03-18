/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DENOISE_H
#define VOXEL_DENOISE_H

#include "Filter.h"

#include <string.h>
#include <deque>

namespace Voxel
{
  
/**
 * \addtogroup Flt
 * @{
 */
  
class VOXEL_EXPORT DenoiseFilter: public Filter
{
protected:
   uint _order;
   float _threshold;

   std::deque<Vector<ByteType>> _ampHistory;
   std::deque<Vector<ByteType>> _phaseHistory;
   std::deque<Vector<ByteType>> _ambHistory;
   std::deque<Vector<ByteType>> _flagsHistory;

   FrameSize _size;
  
   template <typename T>
   bool _filter(const T *in, T *out);
  
   template <typename PhaseT, typename AmpT>
   bool _filter2(const FramePtr &in_p, FramePtr &out_p);

   virtual bool _filter(const FramePtr &in, FramePtr &out);
  
   virtual void _onSet(const FilterParameterPtr &f);
  
public:
   DenoiseFilter(uint order = 3, float threshold = 10000);
  
   virtual void reset();
  
   virtual ~DenoiseFilter() {}
};
/**
 * @}
 */

}
#endif // VOXEL_DENOISE_H
