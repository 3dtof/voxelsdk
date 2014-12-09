/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TIME_H
#define VOXEL_TIME_H

#include <time.h>
#include "Common.h"

namespace Voxel
{
  
class Time
{
  TimeStampType _realTimeStart = 0, _monoticStart = 0;
  
  bool _initialized = false;
  
public:
  Time()
  {
    init();
  }
  
  inline bool isInitialized() { return _initialized; }
  
  bool init();
  
  // Assumption: MONOTIC clock's epoch began recently while REALTIME's epoch began a long time ago (~44 years ago)
  TimeStampType convertToRealTime(TimeStampType l);
  
  TimeStampType getCurentRealTime();
};


}

#endif // VOXEL_TIME_H
