/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "DFT.h"

#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include <iomanip>
#include <fstream>

using namespace Voxel;

enum Options
{
  INPUT_FILE_NAME = 0,
  OUTPUT_FILE_NAME = 1
};

class BMP
{
  unsigned char _info[54];
  
  Vector<char> _fullHeader;
  
public:
  
  int width, height;
  
  bool read(const String &filename, Vector<int16_t> &data)
  {
    int i;
    InputFileStream f(filename, std::ios::in | std::ios::binary);
    
    if(!f.good())
    {
      logger(LOG_ERROR) << "Failed to open '" << filename << "'" << std::endl;
      return -1;
    }
    
    f.read((char *)_info, sizeof(char)*54); // read the 54-byte header
    
    int offset = *(int *)&_info[10];
    // extract image height and width from header
    width = *(int *)&_info[18];
    height = *(int *)&_info[22];
    
    if(offset < 54)
    {
      logger(LOG_ERROR) << "Wrong file format. Got offset = " << offset << std::endl;
      return -1;
    }
    
    _fullHeader.resize(offset - 54);
    
    f.read((char *)_fullHeader.data(), sizeof(char)*_fullHeader.size());
    
    
    if(width < 0 || height < 0)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Negative values for width or height is invalid" << std::endl;
      return false;
    }
    
    if(width > 10000 || height > 10000)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Width or height is too high" << std::endl;
      return false;
    }
    
    int size = width*height;
    data.resize(size); // allocate 2 bytes per pixel
    
    short int pixelWidth;
    
    pixelWidth = *(short int *)&_info[28];
    
    Vector<unsigned char> d;
    
    if(pixelWidth == 24)
    {
      d.resize(size*3);
      
      f.read((char *)d.data(), sizeof(unsigned char)*size*3); // read the rest of the data at once
      
      for(i = 0; i < size; i++)
        data[i] = (d[3*i] + d[3*i + 1] + d[3*i + 2])/3;
    }
    else if(pixelWidth == 8)
    {
      d.resize(size);
      
      f.read((char *)d.data(), sizeof(unsigned char)*size); // read the rest of the data at once
      
      for(i = 0; i < size; i++)
        data[i] = d[i];
    }
    else
    {
      logger(LOG_ERROR) << "Unsupported pixel width = " << pixelWidth << std::endl;
      return false;
    }
    
    f.close();
    return true;
  }
  
  bool write(const String &filename, Vector<int16_t> &data)
  {
    int i;
    OutputFileStream f(filename, std::ios::out | std::ios::binary);
    
    if(!f.good())
    {
      logger(LOG_ERROR) << "Failed to open '" << filename << "'" << std::endl;
      return -1;
    }
    
    f.write((const char *)_info, sizeof(unsigned char)*54);
    f.write((const char *)_fullHeader.data(), sizeof(unsigned char)*_fullHeader.size());
    
    // extract image height and width from header
    int width = *(int*)&_info[18];
    int height = *(int*)&_info[22];
    
    if(width < 0 || height < 0)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Negative values for width or height is invalid" << std::endl;
      return false;
    }
    
    if(width > 10000 || height > 10000)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Width or height is too high" << std::endl;
      return false;
    }
    
    if(data.size() != width*height)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Expected data size to be " << width*height << "bytes, got " << data.size() << " bytes." << std::endl; 
      return false;
    }
    
    int size = width*height;
    
    short int pixelWidth;
    
    pixelWidth = *(short int *)&_info[28];
    
    Vector<unsigned char> d;
    
    if(pixelWidth == 24)
    {
      d.resize(size*3);
      
      for(i = 0; i < size; i++)
      {
        unsigned char c;
        
        if(data[i] > 0)
          c = data[i];
        else
          c = 0;
        
        d[3*i] = c;
        d[3*i + 1] = c;
        d[3*i + 2] = c;
      }
      
      f.write((const char *)d.data(), sizeof(unsigned char)*size*3); // read the rest of the data at once
    }
    else if(pixelWidth == 8)
    {
      d.resize(size);
      
      for(i = 0; i < size; i++)
      {
        unsigned char c;
        
        if(data[i] > 0)
          c = data[i];
        else
          c = 0;
        
        d[i] = c;
      }
      
      f.write((const char *)d.data(), sizeof(unsigned char)*size); // read the rest of the data at once
    }
    else
    {
      logger(LOG_ERROR) << "Unsupported pixel width = " << pixelWidth << std::endl;
    }
    
    f.close();
    
    return true;
  }
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { INPUT_FILE_NAME,         "-i", SO_REQ_SEP, "Input image file name [Only BMP supported]"}, 
  { OUTPUT_FILE_NAME,        "-o", SO_REQ_SEP, "Output image file name [Only BMP supported]"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "DFTTest v1.0" << std::endl;
  
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
  
  String inputFileName, outputFileName;
  
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
      case INPUT_FILE_NAME:
        inputFileName = s.OptionArg();
        break;
        
      case OUTPUT_FILE_NAME:
        outputFileName = s.OptionArg();
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(inputFileName.size() == 0 || outputFileName.size() == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  BMP bmp;
  
  Vector<int16_t> data;
  
  if(!bmp.read(inputFileName, data))
  {
    logger(LOG_ERROR) << "Failed to read '" << inputFileName << "'" << std::endl;
    return -1;
  }
  
  Complex2D c;
  
  c.resize(bmp.height);
  
  for(auto i = 0; i < bmp.height; i++)
  {
    c[i].resize(bmp.width);
    
    for(auto j = 0; j < bmp.width; j++)
    {
      c[i][j] = data[i*bmp.width + j];
    }
  }
  
  std::cout << "Width = " << bmp.width << ", height = " << bmp.height << std::endl;
  
  DFT dft;
  
  if(!dft.DFT2D(c, DFT::FORWARD))
  {
    logger(LOG_ERROR) << "Failed to compute DFT of '" << inputFileName << "'" << std::endl;
    return -1;
  }
  
  if(!dft.DFT2D(c, DFT::REVERSE))
  {
    logger(LOG_ERROR) << "Failed to compute IDFT of '" << inputFileName << "'" << std::endl;
    return -1;
  }
  
  for(auto i = 0; i < bmp.height; i++)
  {
    for(auto j = 0; j < bmp.width; j++)
    {
      int v = abs(c[i][j]);
      if(v > 255)
        data[i*bmp.width + j] = 255;
      else
        data[i*bmp.width + j] = v;
    }
  }
  
  if(!bmp.write(outputFileName, data))
  {
    logger(LOG_ERROR) << "Failed to write '" << outputFileName << "'" << std::endl;
    return -1;
  }
  
  return 0;
}