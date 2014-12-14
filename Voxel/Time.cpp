/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Time.h"
#include "Logger.h"

namespace Voxel
{
  
bool Time::init()
{
  struct timespec t1, t2;
  
  int r1, r2;
  
  r1 = clock_gettime(CLOCK_REALTIME, &t1);
  r2 = clock_gettime(CLOCK_MONOTONIC, &t2);
  
  if(r1 == -1 || r2 == -1)
  {
    logger(ERROR) << "Time: Failed to get current time." << std::endl;
    return _initialized = false;
  }
  
  _realTimeStart = t1.tv_sec*1000000L + t1.tv_nsec/1000;
  _monoticStart = t2.tv_sec*1000000L + t2.tv_nsec/1000;
  
  return _initialized = true;
}

TimeStampType Time::convertToRealTime(TimeStampType l)
{
  if(l > _realTimeStart) // Is it already real-time?
    return l;
  
  return (l - _monoticStart) + _realTimeStart;
}

TimeStampType Time::getCurentRealTime()
{
  struct timespec tp;
  if(clock_gettime(CLOCK_REALTIME, &tp) == -1)
    return 0;
  else
    return tp.tv_sec*1000000L + tp.tv_nsec/1000;
}

}