/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Logger.h"

namespace Voxel
{
  
Logger logger(WARNING);

Logger &endl(Logger &l)
{
  l.getStream() << std::endl;
  return l;
}
  
}