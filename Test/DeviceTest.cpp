/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Device.h"
#include "Logger.h"

using namespace Voxel;

int main()
{
  log.setDefaultLogLevel(ERROR);
  
  Vector<DevicePtr> devices = DeviceScanner::scan();
  
  for(auto i = 0; i < devices.size(); i++)
    std::cout << devices[i]->id() << std::endl;
  
  return 0;
}