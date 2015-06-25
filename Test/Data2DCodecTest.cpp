/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "Data2DCodec.h"

#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>

using namespace Voxel;

enum Options
{
  PHASE_OFFSET_FILE_NAME = 0,
  ROWS = 1,
  COLUMNS = 2,
  OUTPUT_FILE_NAME = 3
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { PHASE_OFFSET_FILE_NAME,  "-p", SO_REQ_SEP, "Phase offset input file name"}, 
  { ROWS,                    "-r", SO_REQ_SEP, "Number of rows"},
  { COLUMNS,                 "-c", SO_REQ_SEP, "Number of columns"},
  { OUTPUT_FILE_NAME,        "-o", SO_REQ_SEP, "Output file name"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "Data2DCodecTest v1.0" << std::endl;
  
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
  
  logger.setDefaultLogLevel(LOG_DEBUG);
  
  String phaseOffsetFileName, outputFileName;
  
  uint rows = 0, columns = 0;
  
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
      case ROWS:
        rows = (uint16_t)strtol(s.OptionArg(), &endptr, 10);
        break;
        
      case COLUMNS:
        columns = (uint16_t)strtol(s.OptionArg(), &endptr, 10);
        break;
        
      case PHASE_OFFSET_FILE_NAME:
        phaseOffsetFileName = s.OptionArg();
        break;
        
      case OUTPUT_FILE_NAME:
        outputFileName = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(phaseOffsetFileName.size() == 0 || outputFileName.size() == 0 || rows == 0 || columns == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  InputFileStream p(phaseOffsetFileName, std::ios::binary | std::ios::ate);
  
  if(!p.good())
  {
    logger(LOG_ERROR) << "Failed to open '" << phaseOffsetFileName << "'" << std::endl;
    return -1;
  }
  
  
  Data2DCodec::Array2D phaseOffsets;
  
  size_t size = p.tellg();
  
  phaseOffsets.resize(size/2); // int16_t data
  
  p.seekg(0, std::ios::beg);
  p.clear();
  
  if(!p.good())
    return -1;
  
  p.read((char *)phaseOffsets.data(), size);
  
  p.close();
  
  Data2DCodec dc;
  
  Data2DCodec::ByteArray output;
  
  if(!dc.compress(phaseOffsets, rows, columns, output))
  {
    logger(LOG_ERROR) << "Failed to compress phase offset data" << std::endl;
    return -1;
  }
  
  std::cout << "Total number of bytes in compressed format = " << output.size() << std::endl;
  
  OutputFileStream out(outputFileName, std::ios::out | std::ios::binary);
  
  if(!out.good())
  {
    logger(LOG_ERROR) << "Could not open output file '" << outputFileName << "'" << std::endl;
    return -1;
  }
  
  out.write((const char *)output.data(), output.size());
  
  out.close();
  
  return 0;
}