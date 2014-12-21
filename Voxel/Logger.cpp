/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Logger.h"

namespace Voxel
{
  
const String Logger::_logLevelNames[5] = { "CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG" };
  
Logger logger(LOG_WARNING);

Logger &endl(Logger &l)
{
  l.getStream() << std::endl;
  return l;
}
  
}