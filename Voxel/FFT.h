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

#include <Common.h>

namespace Voxel
{
  
/**
 * \defgroup Flt Filter related classes
 * @{
 */

  
typedef Vector<Vector<ComplexDouble>> Complex2D;

/**
 *  Perform a 2D FFT inplace given a complex 2D array
 *  The direction dir, 1 for forward, -1 for reverse
 *  The size of the array (nx,ny)
 *  Return false if there are memory problems or
 *     the dimensions are not powers of 2
 */
bool FFT2D(Complex2D &c, int dir);

/**
 *  This computes an in-place complex-to-complex FFT
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
bool FFT(int dir,int m,double *x,double *y);

/**
 *  Calculate the closest but lower power of two of a number
 *  twopm = 2**m <= n
 *  Return TRUE if 2**m == n
 */
int Powerof2(int n,int *m,int *twopm);

/**
 * @}
 */

  
}