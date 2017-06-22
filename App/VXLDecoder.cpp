/*
 * StreamDecoder.cpp
 *
 *  Created on: Jun 21, 2017
 *      Author: a0229967
 */

/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */


#include "CameraSystem.h"

#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include "UVCStreamer.h"
#include <iomanip>
#include <fstream>
#include <iostream>

using namespace Voxel;

enum Options
{
   VXL_FILE = 0,
   FRAME_TYPE = 1,
   OUTPUT_FILE = 2
};

Vector<CSimpleOpt::SOption> argumentSpecifications =
{
  { VXL_FILE,       "-v", SO_REQ_SEP, "Name of the file to dump extracted frames"},
  { FRAME_TYPE,     "-t", SO_REQ_SEP, "Frame type to be saved: raw, amplitude, phase, ambient, flags, depth, pointcloud"},
  { OUTPUT_FILE,     "-o", SO_REQ_SEP, "Output filename. If all is selected, <filename_type> would be the output file" },
    SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "VXLDecoder" << std::endl;

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
  String vxlFileName, frameType, outputFile;

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
      case VXL_FILE:
        vxlFileName = s.OptionArg();
        break;

      case FRAME_TYPE:
        frameType = s.OptionArg();
        break;

      case OUTPUT_FILE:
        outputFile = s.OptionArg();
        break;

      default:
        help();
        break;
    };
  }

  if(vxlFileName.size() == 0 || frameType.size() == 0 || outputFile.size() == 0)
  {
    logger(LOG_ERROR) << "Required arguments missing." << std::endl;
    help();
    return -1;
  }

  CameraSystem sys;

  FrameStreamReader r(vxlFileName, sys);

  if(!r.isStreamGood())
  {
    logger(LOG_ERROR) << "File could not be opened for reading" << std::endl;
    return -1;
  }

  std::cout << "Number of data frames in stream = " << r.size() << std::endl;

  OutputFileStream f (outputFile, std::ios::binary|std::ios::out);

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
    XYZIPointCloudFrame *pointCloudFrame = dynamic_cast<XYZIPointCloudFrame *>(r.frames[DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME].get());

    if(!rawFrame || !tofFrame || !depthFrame || !pointCloudFrame)
    {
      logger(LOG_ERROR) << "Frame(s) not of valid type" << std::endl;
      continue;
    }

    if (frameType == "raw")
    {
      logger(LOG_INFO) << "Saving raw data for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *)rawFrame->data.data(), rawFrame->data.size());
    }

    if (frameType == "amplitude")
    {
      logger(LOG_INFO) << "Saving amplitude for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *) tofFrame->amplitude(),  tofFrame->amplitudeWordWidth()*tofFrame->size.width * tofFrame->size.height);
    }

    if (frameType == "phase")
    {
      logger(LOG_INFO) << "Saving phase for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *) tofFrame->phase(),  tofFrame->phaseWordWidth()*tofFrame->size.width * tofFrame->size.height);
    }

    if (frameType == "flags")
    {
      logger(LOG_INFO) << "Saving flags for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *) tofFrame->flags(),  tofFrame->flagsWordWidth()*tofFrame->size.width * tofFrame->size.height);
    }

    if (frameType == "ambient")
    {
      logger(LOG_INFO) << "Saving ambient for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *) tofFrame->ambient(),  tofFrame->ambientWordWidth()*tofFrame->size.width * tofFrame->size.height);
    }

    if (frameType == "depth")
    {
      logger(LOG_INFO) << "Saving depth data for frame " << tofFrame->id << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *)depthFrame->depth.data(), sizeof(float)*depthFrame->size.width*depthFrame->size.height);
      f.write((char *)depthFrame->amplitude.data(), sizeof(float)*depthFrame->size.width*depthFrame->size.height);
    }

    if (frameType == "pointcloud")
    {
      logger(LOG_INFO) << "Saving point Cloud data for frame " <<  outputFile << "@" << tofFrame->timestamp << "us" << std::endl;
      f.write((char *)pointCloudFrame->points.data(), sizeof(IntensityPoint)*pointCloudFrame->points.size());
    }
  }

  return 0;
}
