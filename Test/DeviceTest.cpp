/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Device.h"
#include "Logger.h"
#include "USBSystem.h"

using namespace Voxel;

int main()
{
  logger.setDefaultLogLevel(LOG_ERROR);
  
  tVector<DevicePtr> devices = DeviceScanner::scan();
  
  USBSystem usbsys;
  
  for(auto i = 0; i < devices.size(); i++)
  {
    String devNode;
    if(devices[i]->interfaceID() == Device::USB)
      devNode = usbsys.getDeviceNode((USBDevice &)*devices[i]);
    
    std::cout << devices[i]->id() << " -- " << devices[i]->description();
    
    if(devNode.size() > 0)
    {
      std::cout << " (" << devNode << ")";
    }
    
    std::cout << std::endl;
  }
  
  return 0;
}