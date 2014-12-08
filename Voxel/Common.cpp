/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Common.h"
#include <sstream>
#include <iomanip>

#include <sys/time.h>

namespace Voxel
{

String getHex(uint16_t value)
{
  std::ostringstream s;
  s << std::hex << std::setw(4) << std::setfill('0') << value;
  return s.str();
}

TimeStampType getLocalCurrentTime()
{
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

  
}