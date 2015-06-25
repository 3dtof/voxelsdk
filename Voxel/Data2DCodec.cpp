/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 */

#include "Data2DCodec.h"

#include "FFT.h"

#include "Logger.h"
#include "SerializedObject.h"

#define FFT_SUBSET_SIZE 10

namespace Voxel
{

bool Data2DCodec::compress(const Array2D &in, const uint16_t rows, const uint16_t columns, ByteArray &out)
{
  if(in.size() != rows*columns)
  {
    logger(LOG_ERROR) << "Data2DCodec: Invalid input data size. Expected " << rows*columns 
    << " bytes, got " << in.size() << " bytes" << std::endl;
    return false;
  }
  
  if(rows % 2 == 1 || columns % 2 == 1)
  {
    logger(LOG_ERROR) << "Data2DCodec: Invalid input data. This codec is designed for only even number of rows and columns." << std::endl;
    return false;
  }
  
  SerializedObject so;
  so.resize(rows*columns*sizeof(in[0])); // Maximum size needed
  
  so.put((const char *)&rows, sizeof(rows));
  so.put((const char *)&columns, sizeof(columns));
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  Complex2D c;
  
  c.resize(rows);
  
  for(auto i = 0; i < rows; i++)
  {
    c[i].resize(columns);
    
    for(auto j = 0; j < columns; j++)
      c[i][j] = in[i*columns + j];
  }
  
  if(!FFT2D(c, 1)) // Forward
  {
    logger(LOG_ERROR) << "Data2DCodec: Failed to get FFT of input data." << std::endl;
    return false;
  }
  
  for(auto i = 0; i < FFT_SUBSET_SIZE; i++)
  {
    for(auto j = 0; j < FFT_SUBSET_SIZE; j++)
    {
      int16_t v;
      
      v = c[i][j].real()*(1 << 6); // Store 6-decimals
      so.put((const char *)&v, sizeof(v));
      
      v = c[i][j].imag()*(1 << 6); // Store 6-decimals
      so.put((const char *)&v, sizeof(v));
    }
  }
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  for(auto i = 0; i < rows; i++)
  {
    for(auto j = 0; j < columns; j++)
    {
      if((i >= 10 && i < rows - 10) || (j >= 10 && j < rows - 10))
        c[i][j] = 0;
    }
  }
  
  if(!FFT2D(c, -1)) // reverse
  {
    logger(LOG_ERROR) << "Data2DCodec: Failed to get FFT of input data." << std::endl;
    return false;
  }
  
  Vector<int8_t> offsets;
  
  offsets.reserve(rows*columns/10);
  
  uint8_t d;
  
  for(auto i = 0; i < rows; i++)
  {
    for(auto j = 0; j < columns; j++)
    {
      int16_t v = in[i*columns + j] - c[i][j].real();
      
      if(v > 7 || v < -7)
      {
        if(v > 127)
          v = 127;
        else if(v < -128)
          v = -128;
        
        offsets.push_back(v);
        
        if(j % 2 == 0)
          d = 0x8;
        else
        {
          d += 0x80;
        }
      }
      else
      {
        if(j % 2 == 0)
          d = ((uint16_t)v) & 0xF;
        else
          d += (((uint16_t)v) & 0xF) << 4;
      }
      
      if(j % 2 == 1)
        so.put((const char *)&d, sizeof(d));
    }
  }
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  so.put((const char *)offsets.data(), offsets.size());
  
  logger(LOG_DEBUG) << "Data2DCodec: current number of bytes = " << so.currentPutOffset() << std::endl;
  
  out.resize(so.currentPutOffset());
  
  memcpy(out.data(), so.getBytes().data(), out.size());
  return true;
}

bool Data2DCodec::decompress(const ByteArray &in, Array2D &out)
{

}

  
}