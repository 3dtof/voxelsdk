/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 */

#include "Data2DCodec.h"

#include "Logger.h"
#include "SerializedObject.h"

#define MAX_OFFSET_COUNT 5000

namespace Voxel
{
  
class BMP
{
public:
  
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
  
  struct BITMAPFILEHEADER
  {
    char bfType[2];
    uint32_t bfSize;
    uint16_t bfReserved1 = 0, bfReserved2 = 0;
    uint32_t bfOffset;
    
    BITMAPFILEHEADER() { bfType[0] = 'B'; bfType[1] = 'M'; }
  };
  
#pragma pack(pop)   /* restore original alignment from stack */
  
  struct BITMAPINFOHEADER
  {
    uint32_t biSize = 40;
    uint32_t biWidth;
    uint32_t biHeight;
    uint16_t biPlanes = 1;
    uint16_t biBitCount = 24;
    uint32_t biCompression = 0;
    uint32_t biSizeImage = 0;
    uint32_t biXPelsPerMeter = 0;
    uint32_t biYPelsPerMeter = 0;
    uint32_t biClrUsed = 0;
    uint32_t biClrImportant = 0;
  };
  
  bool readGrayscale(const String &filename, Vector<int16_t> &data)
  {
    int i;
    InputFileStream f(filename, std::ios::in | std::ios::binary);
    
    if(!f.good())
    {
      logger(LOG_ERROR) << "Failed to open '" << filename << "'" << std::endl;
      return -1;
    }
    
    unsigned char info[54];
    
    Vector<char> fullHeader;
    
    int width, height;
    
    f.read((char *)info, sizeof(char)*54); // read the 54-byte header
    
    int offset = *(int *)&info[10];
    // extract image height and width from header
    width = *(int *)&info[18];
    height = *(int *)&info[22];
    
    if(offset < 54)
    {
      logger(LOG_ERROR) << "Wrong file format. Got offset = " << offset << std::endl;
      return -1;
    }
    
    fullHeader.resize(offset - 54);
    
    f.read((char *)fullHeader.data(), sizeof(char)*fullHeader.size());
    
    
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
    
    pixelWidth = *(short int *)&info[28];
    
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
  
  bool writeGrayScale(const String &filename, const Vector<int16_t> &data, const uint16_t rows, const uint16_t columns)
  {
    int i;
    
    if(data.size() != rows*columns + 2)
    {
      logger(LOG_ERROR) << "Data2DCodecTest: Expected data size to be " << rows*columns << "bytes, got " << data.size() << " bytes." << std::endl; 
      return false;
    }
    
    OutputFileStream f(filename, std::ios::out | std::ios::binary);
    
    if(!f.good())
    {
      logger(LOG_ERROR) << "Failed to open '" << filename << "'" << std::endl;
      return -1;
    }
    
    BITMAPFILEHEADER bfh;
    BITMAPINFOHEADER bih;
    
    bfh.bfOffset = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
    bfh.bfSize = bfh.bfOffset + rows*columns*3;
    
    bih.biWidth = columns;
    bih.biHeight = rows;
    
    f.write((const char *)&bfh, sizeof(BITMAPFILEHEADER));
    f.write((const char *)&bih, sizeof(BITMAPINFOHEADER));
    
    int size = rows*columns;
    
    short int pixelWidth;
    
    Vector<unsigned char> d;
    d.resize(size*3);
    
    const int16_t *dat = &data[2];
      
    for(i = 0; i < size; i++)
    {
      unsigned char c;
      
      c = abs(dat[i]);
      
      d[3*i] = c;
      d[3*i + 1] = c;
      d[3*i + 2] = c;
    }
    
    f.write((const char *)d.data(), sizeof(unsigned char)*size*3);
    f.close();
    
    return true;
  }
};


bool Data2DCodec::compress(const Array2D &in, const ArrayBool2D &invalidPixels, ByteArray &out)
{
  uint16_t rows = *(uint16_t *)&in[0];
  uint16_t columns = *(uint16_t *)&in[1];
  uint16_t dealiasPhaseMask = *(uint16_t *)&in[2];
  
  const int16_t *inData = &in[3];
  
  logger(LOG_DEBUG) << "Data2DCodec: rows = " << rows << ", columns = " << columns << std::endl;
  
  bool noInvalidPixels = (invalidPixels.size() == 0);
    
  
  if(in.size() != rows*columns + 3 || (!noInvalidPixels && invalidPixels.size() != rows*columns))
  {
    logger(LOG_ERROR) << "Data2DCodec: Invalid input data size. Expected " << rows*columns*2 + 6
    << " bytes, got " << in.size()*2 << " bytes" << std::endl;
    return false;
  }
  
  if(columns % 2 != 0)
  {
    logger(LOG_ERROR) << "Data2DCodec: Invalid input data. This codec is designed only for data with columns which are multiples of 2." << std::endl;
    return false;
  }
  
  SerializedObject so;
  so.resize(rows*columns*sizeof(in[0])*10); // Maximum size needed
  
  uint16_t version = 0;
  so.put((const char *)&version, sizeof(version));
  
  so.put((const char *)&rows, sizeof(rows));
  so.put((const char *)&columns, sizeof(columns));
  so.put((const char *)&dealiasPhaseMask, sizeof(dealiasPhaseMask));
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  uint16_t octetRows = (rows + 7)/8;
  uint16_t octetColumns = (columns + 7)/8;
  
  Vector<int32_t> averages(octetRows*octetColumns);
  Vector<int32_t> averageCount(octetRows*octetColumns);
  
  List<EightBitOffset> offsets;
  
  Vector<int8_t> fourBitOffsets;
  fourBitOffsets.resize(rows*columns);
  
  for(auto i = 0; i < averages.size(); i++)
  {
    averages[i] = 0;
    averageCount[i] = 0;
  }
  
  for(auto i = 0; i < rows; i++)
    for(auto j = 0; j < columns; j++)
      {
        averages[i/8*octetColumns + j/8] += inData[i*columns + j];
        averageCount[i/8*octetColumns + j/8]++;
      }
  
  for(auto i = 0; i < averages.size(); i++)
  {
    averages[i] /= averageCount[i];
    int16_t d = averages[i];
    so.put((const char *)&d, sizeof(d));
  }
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  uint32_t invalidPixelCount = 0;
  
  for(auto i = 0; i < rows; i++)
  {
    for(auto j = 0; j < columns; j++)
    {
      int index = i/8*octetColumns + j/8;
      
      int16_t v = (inData[i*columns + j] - averages[index])/_quantization;
      
      if(!noInvalidPixels && invalidPixels[i*columns + j])
      {
        invalidPixelCount++;
        v = 0;
      }
      
      if(v > 7 || v < -7)
      {
        if(v > 127)
          v = 127;
        else if(v < -128)
          v = -128;
        
        offsets.push_back(EightBitOffset(i, j, v, offsets.size()));
        
        fourBitOffsets[i*columns + j] = 8;
      }
      else
        fourBitOffsets[i*columns + j] = v;
    }
  }
  
  int midRow = rows/2, midColumn = columns/2;
  
  uint32_t offsetCount = offsets.size();
  
  if(offsets.size() > MAX_OFFSET_COUNT)
  {
    // Ignore extra offsets near the edges
    offsets.sort([&](const EightBitOffset &a, const EightBitOffset &b) -> bool  { 
        return (a.r - midRow)*(a.r - midRow) + (a.c - midColumn)*(a.c - midColumn) < 
            (b.r - midRow)*(b.r - midRow) + (b.c - midColumn)*(b.c - midColumn);
      });
    
    auto o = offsets.begin();
    std::advance(o, MAX_OFFSET_COUNT);
    
    for(; o != offsets.end(); o++)
    {
      if(o->offset > 0)
        fourBitOffsets[o->r*columns + o->c] = 7;
      else
        fourBitOffsets[o->r*columns + o->c] = -7;
    }
    
    o = offsets.begin();
    std::advance(o, MAX_OFFSET_COUNT);
    offsets.erase(o, offsets.end());
    
    // Reorder in index order
    offsets.sort([&](const EightBitOffset &a, const EightBitOffset &b) -> bool  { 
      return a.index < b.index;
    });
  }
  
  for(auto i = 0; i < fourBitOffsets.size(); i += 2)
  {
    uint8_t d = (((uint8_t)fourBitOffsets[i]) & 0xF) + ((((uint8_t)fourBitOffsets[i + 1]) & 0xF) << 4);
    
    so.put((const char *)&d, 1);
  }
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  for(auto o = offsets.begin(); o != offsets.end(); o++)
  {
    so.put((const char *)&o->offset, 1);
  }
  
  so.put((const char *)&offsetCount, sizeof(offsetCount));
  so.put((const char *)&invalidPixelCount, sizeof(invalidPixelCount));
  
  logger(LOG_DEBUG) << "Data2DCodec: Number of 8-bit offsets = " << offsetCount << std::endl;
  logger(LOG_DEBUG) << "Data2DCodec: Number of invalid pixels = " << invalidPixelCount << std::endl;
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  out.resize(so.currentPutOffset());
  
  memcpy(out.data(), so.getBytes().data(), out.size());
  return true;
}

bool Data2DCodec::writeGrayBMPImage(const String &fileName, const Array2D &a, const uint16_t rows, const uint16_t columns)
{
  BMP bmp;
  
  return bmp.writeGrayScale(fileName, a, rows, columns);
}



bool Data2DCodec::decompress(const ByteArray &in, Array2D &out)
{
  SerializedObject so;
  so.resize(in.size()); // Maximum size needed
  so.put((const char *)in.data(), in.size()*sizeof(in[0]));
  
  uint16_t version;
  so.get((char *)&version, sizeof(version));
  
  uint16_t rows, columns, dealiasPhaseMask;
  
  so.get((char *)&rows, sizeof(rows));
  so.get((char *)&columns, sizeof(columns));
  so.get((char *)&dealiasPhaseMask, sizeof(dealiasPhaseMask));
  
  uint16_t octetRows = (rows + 7)/8;
  uint16_t octetColumns = (columns + 7)/8;
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentGetOffset() << std::endl;
  
  if(columns % 2 != 0)
  {
    logger(LOG_ERROR) << "Data2DCodec: Invalid input data. This codec is designed only for data with columns which are multiples of 2." << std::endl;
    return false;
  }
  
  
  logger(LOG_DEBUG) << "Data2DCodec: rows = " << rows << ", columns = " << columns << std::endl;
  
  Vector<int16_t> averages(octetRows*octetColumns);
  
  so.get((char *)averages.data(), averages.size()*sizeof(averages[0]));
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentGetOffset() << std::endl;
  
  out.resize(rows*columns + 3);
  
  *(uint16_t *)&out[0] = rows;
  *(uint16_t *)&out[1] = columns;
  *(uint16_t *)&out[2] = dealiasPhaseMask;
  
  int16_t *outData = &out[3];
  
  for(auto i = 0; i < rows; i++)
  {
    for(auto j = 0; j < columns; j++)
    {
      outData[i*columns + j] = averages[i/8*octetColumns + j/8];
    }
  }
  
  //writeGrayBMPImage("trialI0.bmp", out, rows, columns);
  
  Vector<uint8_t> fourBitOffsets;
  fourBitOffsets.resize(rows*columns/2);
  
  so.get((char *)fourBitOffsets.data(), fourBitOffsets.size());
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentGetOffset() << std::endl;
  
  uint32_t offsetCount = 0;
  
  for(auto i = 0; i < rows; i++)
  {
    for(auto j = 0; j < columns; j += 2)
    {
      uint8_t d = fourBitOffsets[i*columns/2 + j/2];
      
      int8_t d1 = d & 0xF;
      
      if(d1 != 8)
      {
        if(d1 > 8)
          d1 = d1 - 16;
        outData[i*columns + j] += d1*_quantization;
      }
      else
      {
        int8_t o;
        so.get((char *)&o, sizeof(o));
        
        outData[i*columns + j] += o;
        
        offsetCount++;
      }
      
      int8_t d2 = (d & 0xF0) >> 4;
      
      if(d2 != 8)
      {
        if(d2 > 8)
          d2 = d2 - 16;
        outData[i*columns + j + 1] += d2*_quantization;
      }
      else
      {
        int8_t o;
        so.get((char *)&o, sizeof(o));
        
        outData[i*columns + j + 1] += o*_quantization;
        offsetCount++;
      }
    }
  }
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentGetOffset() << std::endl;
  
  //writeGrayBMPImage("trialI1.bmp", out, rows, columns);
  
  uint32_t totalOffsetCount;
  so.get((char *)&totalOffsetCount, sizeof(totalOffsetCount));
  
  uint32_t invalidPixelCount;
  so.get((char *)&invalidPixelCount, sizeof(invalidPixelCount));
  
  logger(LOG_INFO) << "Data2DCodec: Original number of 8-bit offsets = " << totalOffsetCount << std::endl;
  logger(LOG_INFO) << "Data2DCodec: Number of invalid pixels in original data = " << invalidPixelCount << std::endl;
  logger(LOG_INFO) << "Data2DCodec: Current number of 8-bit offsets = " << offsetCount << std::endl;
  
  return true;
}

  
}