/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Timer.h"
#include "Logger.h"

#include <time.h>

#ifdef WINDOWS
#include <windows.h>
#endif

namespace Voxel
{

// See http://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows
#ifdef WINDOWS
LARGE_INTEGER getFILETIMEoffset()
{
  SYSTEMTIME s;
  FILETIME f;
  LARGE_INTEGER t;

  s.wYear = 1970;
  s.wMonth = 1;
  s.wDay = 1;
  s.wHour = 0;
  s.wMinute = 0;
  s.wSecond = 0;
  s.wMilliseconds = 0;
  SystemTimeToFileTime(&s, &f);
  t.QuadPart = f.dwHighDateTime;
  t.QuadPart <<= 32;
  t.QuadPart |= f.dwLowDateTime;
  return (t);
}

int clock_gettime(TimeStampType &time)
{
  LARGE_INTEGER           t;
  FILETIME            f;
  double                  microseconds;
  static LARGE_INTEGER    offset;
  static double           frequencyToMicroseconds;
  static int              initialized = 0;
  
  if (!initialized) {
    LARGE_INTEGER performanceFrequency;
    initialized = 1;
    offset = getFILETIMEoffset();
    frequencyToMicroseconds = 10.;
  }

  GetSystemTimeAsFileTime(&f);
  t.QuadPart = f.dwHighDateTime;
  t.QuadPart <<= 32;
  t.QuadPart |= f.dwLowDateTime;
  t.QuadPart -= offset.QuadPart;
  
  time = (TimeStampType)t.QuadPart / frequencyToMicroseconds;
  return 0;
}
#endif
  
bool Timer::init()
{
#ifdef LINUX
  struct timespec t1, t2;
  
  int r1, r2;
  
  r1 = clock_gettime(CLOCK_REALTIME, &t1);
  r2 = clock_gettime(CLOCK_MONOTONIC, &t2);
  
  if(r1 == -1 || r2 == -1)
  {
    logger(LOG_ERROR) << "Time: Failed to get current time." << std::endl;
    return _initialized = false;
  }
  
  _realTimeStart = t1.tv_sec*1000000L + t1.tv_nsec/1000;
  _monoticStart = t2.tv_sec*1000000L + t2.tv_nsec/1000;
#elif defined(WINDOWS)
  clock_gettime(_realTimeStart);
  _monoticStart = _realTimeStart;
#endif
  return _initialized = true;
}

TimeStampType Timer::convertToRealTime(TimeStampType l)
{
  if(l > _realTimeStart) // Is it already real-time?
    return l;
  
  return (l - _monoticStart) + _realTimeStart;
}

TimeStampType Timer::getCurentRealTime()
{
#ifdef LINUX
  struct timespec tp;
  if(clock_gettime(CLOCK_REALTIME, &tp) == -1)
    return 0;
  else
    return tp.tv_sec*1000000L + tp.tv_nsec/1000;
#elif defined(WINDOWS)
  TimeStampType t;
  clock_gettime(t);
  return t;
#endif
}

}