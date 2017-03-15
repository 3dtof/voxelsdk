/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include <Common.h>

namespace Voxel
{
  
class VOXEL_EXPORT Convolve2D
{
protected:
  tVector<float> _coefficients;
  
  SizeType _rows, _columns;
  
public:
  Convolve2D(const tVector<float> &coefficients, SizeType rows, SizeType columns);
  
  bool convolve(const tVector<float> &in, SizeType rows, SizeType columns, tVector<float> &out);
  
};
  
  
}