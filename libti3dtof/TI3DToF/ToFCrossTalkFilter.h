/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Filter/Filter.h"

namespace Voxel
{

class ToFCrossTalkFilter: public Filter
{
  virtual bool _filter(const FramePtr& in, FramePtr& out);
  
  template <typename T>
  bool _filter(const T *amplitudeIn, const T *phaseIn, T *amplitudeOut, T *phaseOut);
  
  virtual void _onSet(const FilterParameterPtr& f) {} // No parameters
  
  FrameSize _size;
  
  uint32_t _maxPhaseRange;
  float _phaseToAngleFactor, _angleToPhaseFactor;
  
  Vector<Complex> _amplitudePhase;
  
  Vector<Complex> _coefficients;

public:
  ToFCrossTalkFilter();
  
  bool readCoefficients(const String &coefficients);
  
  bool setMaxPhaseRange(uint32_t maxPhaseRange);
  virtual void reset();
  
  virtual ~ToFCrossTalkFilter() {}
};

typedef Ptr<ToFCrossTalkFilter> ToFCrossTalkFilterPtr;
  
}