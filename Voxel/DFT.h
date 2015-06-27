/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 * Borrowed from: DFT (Discrete Fourier Transform) and FFT (Fast Fourier Transform) 
 * Written by Paul Bourke, June 1993. WWW - http://paulbourke.net/miscellaneous/dft/
 * 
 * Modifications done for datatypes.
 * 
 */

#ifndef VOXEL_DFT_H
#define VOXEL_DFT_H

#include <Common.h>

namespace Voxel
{
  
/**
 * \defgroup Flt Filter related classes
 * @{
 */

typedef Vector<ComplexDouble> Complex1D;
typedef Vector<Vector<ComplexDouble>> Complex2D;

class VOXEL_EXPORT DFT
{
public:
  enum Direction
  {
    FORWARD = 1,
    REVERSE = -1
  };
  
protected:
  Complex2D _internal;
  
  Map<SizeType, Complex2D> _expTable;
  
  void _computeExpTable(SizeType size, Direction dir);
  
public:
  /**
  *  Perform a 2D FFT inplace given a complex 2D array
  *  The direction dir, 1 for forward, -1 for reverse
  *  The size of the array (nx,ny)
  *  Return false if there are memory problems or
  *     the dimensions are not powers of 2
  */
  bool DFT2D(Complex2D &data, Direction dir);
  
  /**
  *  This computes an in-place complex-to-complex DFT
  *  x and y are the real and imaginary arrays of 2^m points.
  *  dir =  1 gives forward transform
  *  dir = -1 gives reverse transform
  * 
  *    Formula: forward
  *                 N-1
  *                 ---
  *             1   \          - j k 2 pi n / N
  *     X(n) = ---   >   x(k) e                    = forward transform
  *             N   /                                n=0..N-1
  *                 ---
  *                 k=0
  * 
  *     Formula: reverse
  *                 N-1
  *                 ---
  *                 \          j k 2 pi n / N
  *     X(n) =       >   x(k) e                    = forward transform
  *                 /                                n=0..N-1
  *                 ---
  *                 k=0
  */
  bool DFT1D(const Complex1D &in, Complex1D &out, Direction dir);
};

/**
 * @}
 */

  
}

#endif