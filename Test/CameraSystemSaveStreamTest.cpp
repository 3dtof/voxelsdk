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

tVector<CSimpleOpt::SOption> argumentSpecifications = 
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
  std::cout << "CameraSystemSaveStreamTest v1.0" << std::endl;
  
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
  
  logger.setDefaultLogLevel(LOG_INFO);
  
  uint16_t vid = 0;
  
  tVector<uint16_t> pids;
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
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  CameraSystem sys;
  
  // Get all valid detected devices
  const tVector<DevicePtr> &devices = sys.scan();
  
  DevicePtr toConnect;
  
  std::cout << "Detected devices: " << std::endl;
  for(auto &d: devices)
  {
    std::cout << d->id() << std::endl;
    
    if(d->interfaceID() == Device::USB)
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
  
  FilterPtr p = sys.createFilter("Voxel::IIRFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  
  if(!p)
  {
    logger(LOG_ERROR) << "Failed to get IIRFilter" << std::endl;
    return -1;
  }
  
  depthCamera->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  
  p = sys.createFilter("Voxel::IIRFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
  
  if(!p)
  {
    logger(LOG_ERROR) << "Failed to get IIRFilter" << std::endl;
    return -1;
  }
  
  p->set("gain", 0.2f);
  
  depthCamera->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED, 0);
  
  int count = 0;
  
  TimeStampType lastTimeStamp = 0;
  
//    depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_UNPROCESSED, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
//      const RawDataFrame *d = dynamic_cast<const RawDataFrame *>(&frame);
//      
//      if(!d)
//      {
//        std::cout << "Null frame captured? or not of type RawDataFrame" << std::endl;
//        return;
//      }
//      
//      std::cout << "Capture frame " << d->id << "@" << d->timestamp;
//      
//      if(lastTimeStamp != 0)
//        std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
//      
//      std::cout << std::endl;
//      
//      lastTimeStamp = d->timestamp;
//      
//      f.write((char *)d->data.data(), d->data.size());
//      
//      count++;
//      
//      if(count >= 100)
//        dc.stop();
//    });
  
  depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
    
    const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);
    
    if(!d)
    {
      std::cout << "Null frame captured? or not of type ToFRawFrame" << std::endl;
      return;
    }
    
    std::cout << "Capture frame " << d->id << "@" << d->timestamp;
    
    if(lastTimeStamp != 0)
      std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
      
    std::cout << std::endl;
    
    lastTimeStamp = d->timestamp;
    
    if(d->id == 0 && !depthCamera->saveFrameStream(dumpFileName))
    {
      logger(LOG_ERROR) << "Failed to open '" << dumpFileName << "'" << std::endl;
    }

    count++;
    
    if(count >= frameCount)
    {
      dc.stop();
    }
  });
  
//   depthCamera->registerCallback(DepthCamera::FRAME_DEPTH_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
//     const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);
//     
//     if(!d)
//     {
//       std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
//       return;
//     }
//     
//     std::cout << "Capture frame " << d->id << "@" << d->timestamp;
//     
//     if(lastTimeStamp != 0)
//       std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
//     
//     std::cout << std::endl;
//     
//     lastTimeStamp = d->timestamp;
//     
//     f.write((char *)d->depth.data(), sizeof(float)*d->size.width*d->size.height);
//     f.write((char *)d->amplitude.data(), sizeof(float)*d->size.width*d->size.height);
//     
//     count++;
//     
//     if(count >= 100)
//       dc.stop();
//   });
  
  //depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
  //  const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
  //  
  //  if(!d)
  //  {
  //    std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
  //    return;
  //  }
  //  
  //  std::cout << "Capture frame " << d->id << "@" << d->timestamp;
  //  
  //  if(lastTimeStamp != 0)
  //    std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
  //  
  //  std::cout << std::endl;
  //  
  //  lastTimeStamp = d->timestamp;
  //  
  //  f.write((char *)d->points.data(), sizeof(IntensityPoint)*d->points.size());
  //  
  //  count++;
  //  
  //  if(count >= 100)
  //    dc.stop();
  //});
  
  
  if(depthCamera->start())
  {
    FrameRate r;
    if(depthCamera->getFrameRate(r))
      logger(LOG_INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
    depthCamera->wait();
  }
  else
    logger(LOG_ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;

  return 0;
}