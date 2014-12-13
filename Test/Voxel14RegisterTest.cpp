/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Downloader.h"
#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <Parameter.h>
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
  DATA = 6,
  LENGTH = 7,
  LSB = 8,
  MSB = 9
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
  { LENGTH,       "-t", SO_REQ_SEP, "Length of the register in bits (integer) [default = 24]"},
  { LSB,          "-l", SO_REQ_SEP, "LSB of data (integer) [default = 0]"},
  { MSB,          "-b", SO_REQ_SEP, "MSB of data (integer) [default = 23]"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "Voxel14RegisterTest v1.0" << std::endl;
  
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
  
  uint8_t lsb = 0, msb = 23, registerLength = 24;
  
  
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
        
      case LENGTH:
        registerLength = (uint8_t)strtol(s.OptionArg(), &endptr, 10);
        break;
        
      case LSB:
        lsb = (uint8_t)strtol(s.OptionArg(), &endptr, 10);
        break;
        
      case MSB:
        msb = (uint8_t)strtol(s.OptionArg(), &endptr, 10);
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
  
  UnsignedIntegerParameter p(programmer, "", "", address, registerLength, msb, lsb, 0, (1 << (msb - lsb + 1)) - 1, 0, "", "");
  
  if(write)
  {
    uint32_t value;
    programmer.readRegister(address, value);
    std::cout << "Register (before set) @0x" << std::hex << address << " = 0x" << std::hex << value << std::endl;
    p.set(data);
    programmer.readRegister(address, value);
    std::cout << "Register (after set) @0x" << std::hex << address << " = 0x" << std::hex << value << std::endl;
    //programmer.writeRegister(address, data);
  }
  else
  {
    uint value;
    if(p.get(value, true))
      std::cout << "Register @0x" << std::hex << address << " = 0x" << std::hex << value << std::endl;
    else
      log(ERROR) << "Could not read register @0x" << std::hex << address << std::endl;
  }
  
  return 0;
}