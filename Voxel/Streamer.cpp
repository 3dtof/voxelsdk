/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Streamer.h"

namespace Voxel
{
  
bool Streamer::start()
{
  if(!isInitialized() || !_start())
    return false;
  
  _currentID = 0;
  
  return _isRunning = true;
}

bool Streamer::capture(RawDataFramePtr &p)
{
  if(!isInitialized() || !isRunning() || !_capture(p))
    return false;
  
  return true;
}


bool Streamer::stop()
{
  if(!isInitialized() || !isRunning())
    return false;
  
  _isRunning = false;
  
  if(!_stop())
    return false;
  
  return true;
}

Streamer::~Streamer()
{
}

}