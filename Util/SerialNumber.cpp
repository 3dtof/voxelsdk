/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "CameraSystem.h"

#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include "UVCStreamer.h"
#include <iomanip>
#include <fstream>

using namespace Voxel;

enum Options
{
  VENDOR_ID = 0,
  PRODUCT_ID = 1,
  SERIAL_NUMBER = 2,
};

tVector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,    "-v", SO_REQ_SEP, "Vendor ID of the USB device (hexadecimal)"}, // Only worker count is needed here
  { PRODUCT_ID,   "-p", SO_REQ_SEP, "Comma separated list of Product IDs of the USB devices (hexadecimal)"},
  { SERIAL_NUMBER,"-s", SO_REQ_SEP, "Set serial number of the USB device to this (string)"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "SerialNumber v1.0" << std::endl;
  
  CSimpleOpt::SOption *option = argumentSpecifications.data();
  
  while(option->nId >= 0)
  {
    std::cout << option->pszArg << " " << option->helpInfo << std::endl;
    option++;
  }
}


int main(int argc, char *argv[])
{
  CSimpleOpt s(argc, argv, argumentSpecifications);
  
  logger.setDefaultLogLevel(LOG_ERROR);
  
  uint16_t vid = 0;
  
  tVector<uint16_t> pids;
  String serialNumber;

  char *endptr;
  
  while (s.Next())
  {
    if (s.LastError() != SO_SUCCESS)
    {
      std::cout << s.GetLastErrorText(s.LastError()) << ": '" << s.OptionText() << "' (use -h to get command line help)" << std::endl;
      help();
      return -1;
    }
    
    //std::cout << s.OptionId() << ": " << s.OptionArg() << std::endl;
    
    tVector<String> splits;
    switch (s.OptionId())
    {
      case VENDOR_ID:
        vid = (uint16_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case PRODUCT_ID:
        split(s.OptionArg(), ',', splits);
        
        for(auto &s1: splits)
          pids.push_back((uint16_t)strtol(s1.c_str(), &endptr, 16));
        
        break;
        
      case SERIAL_NUMBER:
        serialNumber = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(vid == 0 || pids.size() == 0 || pids[0] == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  CameraSystem sys;
  
  // Get all valid detected devices
  const tVector<DevicePtr> &devices = sys.scan();
  
  DevicePtr toConnect;
  
  for(auto &d: devices)
  {
    if(d->interfaceID() == Device::USB)
    {
      USBDevice &usb = (USBDevice &)*d;
      
      if(usb.vendorID() == vid)
      {
        for(auto pid: pids)
          if(usb.productID() == pid)
            toConnect = d;
      }
    }
  }
  
  if(!toConnect)
  {
    logger(LOG_ERROR) << "No valid device found for the specified VID:PID:serialnumber" << std::endl;
    return -1;
  }
    
  DepthCameraPtr depthCamera = sys.connect(toConnect);
  
  if(!depthCamera)
  {
    logger(LOG_ERROR) << "Could not load depth camera for device " << toConnect->id() << std::endl;
    return -1;
  }

  if(!depthCamera->isInitialized())
  {
    logger(LOG_ERROR) << "Depth camera not initialized for device " << toConnect->id() << std::endl;
    return -1;
  }
  
  std::cout << "Successfully loaded depth camera for device " << toConnect->id() << std::endl;
  
  String serialNumber2;
  
  if(!depthCamera->getSerialNumber(serialNumber2))
  {
    logger(LOG_ERROR) << "Could not get serial number for the depth camera. Exiting." << std::endl;
    return -1;
  }
  
  std::cout << "\nSerial number of the depth camera = " << serialNumber2 << std::endl;
  
  if(serialNumber.size() > 0)
  {
    char ch;
    std::cout << "\nAbout to write '" << serialNumber << "' as serial number..." << std::endl;
    std::cout << "Are you sure you want to change the serial number [y/n]? ";
    std::cin >> ch;
    
    if(ch == 'y' || ch == 'Y')
    {
      std::cout << std::endl << "Writing serial number to depth camera..." << std::endl;
      
      if(!depthCamera->setSerialNumber(serialNumber))
      {
        return -1;
      }
      std::cout << "Done! Please power cycle the board." << std::endl;
    }
    else
    {
      std::cout << std::endl << "Not writing the serial number. Exiting."<< std::endl;
    }
  }
  
  return 0;
}