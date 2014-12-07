/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Downloader.h"
#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <VoxelXUProgrammer.h>

using namespace Voxel;

enum Options
{
  VENDOR_ID = 0,
  PRODUCT_ID = 1,
  SERIAL_NUMBER = 2,
  REGISTER = 3,
  READ = 4,
  WRITE = 5,
  DATA = 6
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,    "-v", SO_REQ_SEP, "Vendor ID of the USB device (hexadecimal)"}, // Only worker count is needed here
  { PRODUCT_ID,   "-p", SO_REQ_SEP, "Product ID of the USB device (hexadecimal)"},
  { SERIAL_NUMBER,"-s", SO_REQ_SEP, "Serial number of the USB device (string)"},
  { REGISTER,     "-r", SO_REQ_SEP, "Register address (hexadecimal)"},
  { READ,         "-i", SO_NONE,    "Read register (specify only one of -i/-o)"},
  { WRITE,        "-o", SO_NONE,    "Write to register (specify only one of -i/-o)"},
  { DATA,         "-d", SO_REQ_SEP, "Data to write to register (hexadecimal)"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "DownloadTester v1.0" << std::endl;
  
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
  
  log.setDefaultLogLevel(INFO);
  
  uint16_t vid = 0, pid = 0;
  String serialNumber;
  uint32_t address = -1, data = -1;
  
  bool write = true; // read -> false, write -> true
  
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
    
    switch (s.OptionId())
    {
      case VENDOR_ID:
        vid = (uint16_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case PRODUCT_ID:
        pid = (uint16_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case SERIAL_NUMBER:
        serialNumber = s.OptionArg();
        break;
        
      case REGISTER:
        address = (uint32_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case DATA:
        data = (uint32_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case READ:
        write = false;
        break;
        
      case WRITE:
        write = true;
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(vid == 0 || pid == 0 || address == -1)
  {
    log(ERROR) << "Required argument missing." << endl;
    help();
    return -1;
  }
  
  DevicePtr ud(new USBDevice(vid, pid, serialNumber));
  
  TI::VoxelXUProgrammer programmer(ud);
  
  if(write)
  {
    programmer.writeRegister(address, data);
  }
  else
  {
    if(programmer.readRegister(address, data))
    {
      std::cout << "Register @0x" << std::hex << address << " = 0x" << std::hex << data << std::endl;
    }
  }
  
  return 0;
}