/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

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
  DUMP_FILE = 3,
  NUM_OF_FRAMES = 4
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,    "-v", SO_REQ_SEP, "Vendor ID of the USB device (hexadecimal)"}, // Only worker count is needed here
  { PRODUCT_ID,   "-p", SO_REQ_SEP, "Product ID of the USB device (hexadecimal)"},
  { SERIAL_NUMBER,"-s", SO_REQ_SEP, "Serial number of the USB device (string)"},
  { DUMP_FILE,    "-f", SO_REQ_SEP, "Name of the file to dump extracted frames"},
  { NUM_OF_FRAMES,"-n", SO_REQ_SEP, "Number of frames to dump [default = 1]"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "UVCStreamerTest v1.0" << std::endl;
  
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
  
  logger.setDefaultLogLevel(INFO);
  
  uint16_t vid = 0, pid = 0;
  String serialNumber;
  String dumpFileName;
  
  int32_t frameCount = 1;
  
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
        
      case DUMP_FILE:
        dumpFileName = s.OptionArg();
        break;
        
      case NUM_OF_FRAMES:
        frameCount = (int32_t)strtol(s.OptionArg(), &endptr, 10);
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(vid == 0 || pid == 0 || dumpFileName.size() == 0)
  {
    logger(ERROR) << "Required argument missing." << endl;
    help();
    return -1;
  }
  
  std::ofstream f(dumpFileName, std::ios::binary | std::ios::out);
  
  if(!f.good())
  {
    logger(ERROR) << "Failed to open '" << dumpFileName << "'" << endl;
    return -1;
  }
  
  DevicePtr ud(new USBDevice(vid, pid, serialNumber));
  
  UVCStreamer streamer(ud);
  
  if(!streamer.isInitialized())
  {
    logger(ERROR) << "UVCStreamer not initialized" << endl;
    return -1;
  }
  
  Vector<VideoMode> videoModes;
  
  if(streamer.getSupportedVideoModes(videoModes))
  {
    std::cout << "Supported video modes" << std::endl;
    
    for(auto i = 0; i < videoModes.size(); i++)
    {
      std::cout << videoModes[i].frameSize.width << "x" << videoModes[i].frameSize.height << "@" << videoModes[i].getFrameRate() << "fps" << std::endl;
    } 
  }
  
  VideoMode c; 
  
  if(streamer.getCurrentVideoMode(c))
    std::cout << "\nCurrent video mode: " << c.frameSize.width << "x" << c.frameSize.height << "@" << c.getFrameRate() << "fps" << std::endl;
  else
    logger(ERROR) << "UVCStreamerTest: Could not get current video mode" << endl;
  
  c.frameSize.width = 320;
  c.frameSize.height = 240;
  
  if(!streamer.setVideoMode(c))
  {
    logger(ERROR) << "Could not set the video mode to 320x240" << std::endl;
    return -1;
  }
  else
    std::cout << "Video mode changed to: " << c.frameSize.width << "x" << c.frameSize.height << "@" << c.getFrameRate() << "fps" << std::endl;
  
  
  if(!streamer.start())
  {
    logger(ERROR) << "UVCStreamer not ready for capture" << endl;
    return -1;
  }
  
  
  RawDataFramePtr p;
  
  for(auto i = 0; i < frameCount; i++)
  {
    if(!streamer.capture(p))
    {
      logger(WARNING) << "UVCStreamer could not capture a frame" << endl;
      i--;
    }
    else
    {
      std::cout << "Capture frame " << p->id << "@" << p->timestamp << " of size = " << p->data.size() << std::endl;
      f.write((char *)p->data.data(), p->data.size());
    }
  }
  
  if(!streamer.stop())
  {
    logger(ERROR) << "UVCStreamer could not be stopped" << endl;
    return -1;
  }
  
  
  return 0;
}