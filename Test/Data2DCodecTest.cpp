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
  OUTPUT_FILE_NAME = 3
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { PHASE_OFFSET_FILE_NAME,  "-p", SO_REQ_SEP, "Phase offset input file name"}, 
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
  
  uint16_t rows = 0, columns = 0;
  
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
  
  if(phaseOffsetFileName.size() == 0 || outputFileName.size() == 0)
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
  Data2DCodec::ArrayBool2D invalidPixels;
  
  size_t size = p.tellg();
  
  size -= 6; // for uint16_t row, column
  
  p.seekg(0, std::ios::beg);
  p.clear();
  
  if(!p.good())
    return -1;
  
  p.read((char *)&rows, sizeof(uint16_t));
  p.read((char *)&columns, sizeof(uint16_t));
  
  p.seekg(0, std::ios::beg);
  p.clear();
  
  if(size == rows*columns*2)
  {
    phaseOffsets.resize(size/2 + 3); // int16_t data
    p.read((char *)phaseOffsets.data(), size + 4);
  }
  else if(size = rows*columns*3) // including invalid pixel boolean 2D array?
  {
    phaseOffsets.resize(size/3 + 3); // int16_t data
    invalidPixels.resize(size/3);
    p.read((char *)phaseOffsets.data(), (size*2)/3 + 6);
    p.read((char *)invalidPixels.data(), size/3);
  }
  
  p.close();
  
  Data2DCodec dc;
  
  Data2DCodec::ByteArray output;
  
  if(!dc.compress(phaseOffsets, invalidPixels, output))
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
  
  if(!dc.decompress(output, phaseOffsets))
  {
    logger(LOG_ERROR) << "Failed to decompress phase offset data" << std::endl;
    return -1;
  }
  
  return 0;
}