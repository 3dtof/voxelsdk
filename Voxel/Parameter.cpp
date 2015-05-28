/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Parameter.h"

namespace Voxel
{
  
  
void Parameter::_computeMask()
{
  if(_msb < _lsb)
    _mask = -1; // fictitious register
    
  _mask = (1 << _registerLength) - 1;  
  for (auto i = _lsb; i <= _msb; i++) 
  {
    _mask -= (1 << i);
  }
}
}