/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Downloader.h"
#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"

using namespace Voxel;

enum Options
{
  VENDOR_ID = 0,
  PRODUCT_ID = 1,
  SERIAL_NUMBER = 2,
  FIRMWARE_FILE = 3
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,    "-v", SO_REQ_SEP, "Vendor ID of the USB device"}, // Only worker count is needed here
  { PRODUCT_ID,   "-p", SO_REQ_SEP, "Product ID of the USB device"},
  { SERIAL_NUMBER,"-s", SO_REQ_SEP, "Product ID of the USB device"},
  { FIRMWARE_FILE,"-f", SO_REQ_SEP, "Firmware file name"},
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
  
  log.setDefaultLogLevel(ERROR);
  
  uint16_t vid = 0, pid = 0;
  String serialNumber;
  String firmwareFileName;
  
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
        
      case FIRMWARE_FILE:
        firmwareFileName = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(vid == 0 || pid == 0 || firmwareFileName.size() == 0)
  {
    log(ERROR) << "Required argument missing." << endl;
    help();
    return -1;
  }
  
  DevicePtr ud(new USBDevice(vid, pid, serialNumber));
  
  USBDownloader d(ud);
  
  if(d.download(firmwareFileName))
  {
    log(INFO) << "Download successful!" << endl;
    return 0;
  }
  else
    return -1;
}