/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <CameraSystem.h>
#include <Common.h>

#include <thread>

#include "CLIManager.h"

int main()
{
  Voxel::logger.setDefaultLogLevel(Voxel::LOG_INFO);
  
  Voxel::CameraSystem sys;
  
  Voxel::CLIManager manager(sys);
  
  manager.run();
}