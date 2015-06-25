/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 */

#include "Common.h"

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
public:
  typedef Vector<ByteType> ByteArray;
  
  typedef Vector<int16_t> Array2D;
  
  Data2DCodec() {}
  
  bool compress(const Array2D& in, const uint16_t rows, const uint16_t columns, ByteArray& out);
  bool decompress(const ByteArray &in, Array2D &out);
  
  virtual ~Data2DCodec() {}
};

/**
 * @}
 */  

  
}