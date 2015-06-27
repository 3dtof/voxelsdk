/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 */

#ifndef VOXEL_DATA_2D_CODEC_H
#define VOXEL_DATA_2D_CODEC_H

#include "Common.h"

#include "DFT.h"

namespace Voxel
{
  
/**
 * \defgroup Flt Filter related classes
 * @{
 */

/**
 * @brief This class is used for compressing and decompressing 16-bit signed data.
 * 
 * Primary application of this is for 2D phase offset data compression to store in 
 * permanent memory of depth camera
 * 
 */
class VOXEL_EXPORT Data2DCodec
{
  struct EightBitOffset
  {
    int r, c;
    int8_t offset;
    
    int index;
    
    EightBitOffset(int r, int c, int8_t offset, int index): r(r), c(c), offset(offset), index(index) {}
  };
  
public:
  typedef Vector<ByteType> ByteArray;
  typedef Vector<uint8_t> ArrayBool2D;
  typedef Vector<int16_t> Array2D;
  
  Data2DCodec() {}
  
  bool compress(const Array2D& in, const ArrayBool2D &invalidPixels, const uint16_t rows, const uint16_t columns, ByteArray& out);
  bool decompress(const ByteArray &in, uint16_t &rows, uint16_t &columns, Array2D &out);
  
  bool writeGrayBMPImage(const String &fileName, const Array2D &a, const uint16_t rows, const uint16_t columns);
  bool writeGrayBMPImage(const String &fileName, const Complex2D &c);
  
  virtual ~Data2DCodec() {}
};

/**
 * @}
 */  

  
}

#endif