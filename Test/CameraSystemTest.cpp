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
  DUMP_FILE = 3,
  NUM_OF_FRAMES = 4
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,    "-v", SO_REQ_SEP, "Vendor ID of the USB device (hexadecimal)"}, // Only worker count is needed here
  { PRODUCT_ID,   "-p", SO_REQ_SEP, "Comma separated list of Product IDs of the USB devices (hexadecimal)"},
  { SERIAL_NUMBER,"-s", SO_REQ_SEP, "Serial number of the USB device (string)"},
  { DUMP_FILE,    "-f", SO_REQ_SEP, "Name of the file to dump extracted frames"},
  { NUM_OF_FRAMES,"-n", SO_REQ_SEP, "Number of frames to dump [default = 1]"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "CameraSystemTest v1.0" << std::endl;
  
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
  
  uint16_t vid = 0;
  
  Vector<uint16_t> pids;
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
    
    Vector<String> splits;
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
  
  if(vid == 0 || pids.size() == 0 || pids[0] == 0 || dumpFileName.size() == 0)
  {
    logger(ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  std::ofstream f(dumpFileName, std::ios::binary | std::ios::out);
  
  if(!f.good())
  {
    logger(ERROR) << "Failed to open '" << dumpFileName << "'" << std::endl;
    return -1;
  }
  
  CameraSystem sys;
  
  // Get all valid detected devices
  const Vector<DevicePtr> &devices = sys.scan();
  
  DevicePtr toConnect;
  
  std::cout << "Detected devices: " << std::endl;
  for(auto &d: devices)
  {
    std::cout << d->id() << std::endl;
    
    if(d->interface() == Device::USB)
    {
      USBDevice &usb = (USBDevice &)*d;
      
      if(usb.vendorID() == vid && (serialNumber.size() == 0 || usb.serialNumber() == serialNumber))
      {
        for(auto pid: pids)
          if(usb.productID() == pid)
            toConnect = d;
      }
    }
  }
  
  if(!toConnect)
  {
    logger(ERROR) << "No valid device found for the specified VID:PID:serialnumber" << std::endl;
    return -1;
  }
    
  DepthCameraPtr depthCamera = sys.connect(toConnect);
  
  if(!depthCamera)
  {
    logger(ERROR) << "Could not load depth camera for device " << toConnect->id() << std::endl;
    return -1;
  }

  if(!depthCamera->isInitialized())
  {
    logger(ERROR) << "Depth camera not initialized for device " << toConnect->id() << std::endl;
    return -1;
  }
  
  std::cout << "Successfully loaded depth camera for device " << toConnect->id() << std::endl;
  
  int count = 0;
  
  TimeStampType lastTimeStamp = 0;
  
  depthCamera->registerCallback(DepthCamera::CALLBACK_RAW_FRAME_UNPROCESSED, [&](DepthCamera &dc, Frame &frame, DepthCamera::FrameCallBackType c) {
    RawDataFrame *d = dynamic_cast<RawDataFrame *>(&frame);
    
    if(!d)
    {
      std::cout << "Null frame captured? or not of type RawDataFrame" << std::endl;
      return;
    }
    
    std::cout << "Capture frame " << d->id << "@" << d->timestamp << " of size = " << d->data.size();
    
    if(lastTimeStamp != 0)
      std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
      
    std::cout << std::endl;
    
    lastTimeStamp = d->timestamp;
      
    f.write((char *)d->data.data(), d->data.size());
    
    count++;
    
    if(count >= 100)
      dc.stop();
  });
  
  if(depthCamera->start())
  {
    FrameRate r;
    if(depthCamera->getFrameRate(r))
      logger(INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
    depthCamera->wait();
  }
  else
    logger(ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;
  
  return 0;
}