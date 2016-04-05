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
  DUMP_FILE = 3,
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { DUMP_FILE,    "-f", SO_REQ_SEP, "Name of the file to dump extracted frames"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "CameraSystemReadStreamTest v1.0" << std::endl;
  
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
  
  String dumpFileName;
  
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
      case DUMP_FILE:
        dumpFileName = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(dumpFileName.size() == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  CameraSystem sys;
  
  FrameStreamReader r(dumpFileName, sys);
  
  if(!r.isStreamGood())
  {
    logger(LOG_ERROR) << "File could not be opened for reading" << std::endl;
    return -1;
  }
  
  std::cout << "Number of data frames in stream = " << r.size() << std::endl;
  
  for(auto i = 0; i < r.size(); i++)
  {
    if(!r.readNext())
    {
      logger(LOG_ERROR) << "Could not process frame id = " << i << std::endl;
      continue;
    }
    
    RawDataFrame *rawFrame = dynamic_cast<RawDataFrame *>(r.frames[DepthCamera::FRAME_RAW_FRAME_UNPROCESSED].get());
    ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(r.frames[DepthCamera::FRAME_RAW_FRAME_PROCESSED].get());
    DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(r.frames[DepthCamera::FRAME_DEPTH_FRAME].get());
    PointCloudFrame *pointCloudFrame = dynamic_cast<PointCloudFrame *>(r.frames[DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME].get());
    
    if(!rawFrame || !tofFrame || !depthFrame || !pointCloudFrame)
    {
      logger(LOG_ERROR) << "Frame(s) not of valid type" << std::endl;
      continue;
    }
    
    std::cout << rawFrame->id << "@" << rawFrame->timestamp << std::endl;
    std::cout << "Raw data frame size = " << rawFrame->data.size() << " bytes" << std::endl;
    std::cout << "ToF frame size = " << tofFrame->size.width << "x" << tofFrame->size.height << std::endl;
    std::cout << "Depth frame size = " << depthFrame->size.width << "x" << depthFrame->size.height << std::endl;
    std::cout << "Point cloud frame points = " << pointCloudFrame->size() << std::endl;
  }
  
  return 0;
}