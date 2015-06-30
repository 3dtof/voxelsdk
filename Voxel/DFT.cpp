/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 *
 * Derived from http://paulbourke.net/miscellaneous/dft/
 * with major modifications to avoid too many memory allocations.
 *
 *
 * Reference: DFT (Discrete Fourier Transform) and FFT (Fast Fourier Transform)
 * Written by Paul Bourke, June 1993
 *
 * This is not meant to be an efficient implementation.
 *
 */

#include <malloc.h>
#include "DFT.h"

#include "Logger.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace Voxel
{



// bool FFT2D(Complex2D &c, int dir)
// {
//   if(c.size() == 0)
//     return false;
// 
//   int i, j;
//   int m, twopm;
//   double *real, *imag;
// 
//   int nx = c[0].size();
//   int ny = c.size();
// 
//   /* Transform the rows */
//   real = (double *)malloc(nx * sizeof(double));
//   imag = (double *)malloc(nx * sizeof(double));
// 
//   if(real == NULL || imag == NULL)
//   {
//     logger(LOG_ERROR) << "FFT2D: Failed to allocate memory" << std::endl;
//     return false;
//   }
// 
//   if(!Powerof2(nx, &m, &twopm) || twopm != nx)
//   {
//     logger(LOG_ERROR) << "FFT2D: Failed to get nearest power of 2 of '" << nx << "'" << std::endl;
//     return false;
//   }
// 
//   for(j = 0; j < ny; j++)
//   {
//     for(i = 0; i < nx; i++)
//     {
//       real[i] = c[i][j].real();
//       imag[i] = c[i][j].imag();
//     }
// 
//     FFT(dir, m, real, imag);
// 
//     for(i = 0; i < nx; i++)
//     {
//       c[i][j].real(real[i]);
//       c[i][j].imag(imag[i]);
//     }
//   }
// 
//   free(real);
//   free(imag);
// 
//   /* Transform the columns */
//   real = (double *)malloc(ny * sizeof(double));
//   imag = (double *)malloc(ny * sizeof(double));
// 
//   if(real == NULL || imag == NULL)
//   {
//     logger(LOG_ERROR) << "FFT2D: Failed to allocate memory" << std::endl;
//     return false;
//   }
// 
//   if(!Powerof2(ny, &m, &twopm) || twopm != ny)
//   {
//     logger(LOG_ERROR) << "FFT2D: Failed to get nearest power of 2 of '" << nx << "'" << std::endl;
//     return false;
//   }
// 
//   for(i = 0; i < nx; i++)
//   {
//     for(j = 0; j < ny; j++)
//     {
//       real[j] = c[i][j].real();
//       imag[j] = c[i][j].imag();
//     }
// 
//     FFT(dir, m, real, imag);
// 
//     for(j = 0; j < ny; j++)
//     {
//       c[i][j].real(real[j]);
//       c[i][j].imag(imag[j]);
//     }
//   }
// 
//   free(real);
//   free(imag);
// 
//   return true;
// }
// 
// bool FFT(int dir, int m, double *x, double *y)
// {
//   long nn, i, i1, j, k, i2, l, l1, l2;
//   double c1, c2, tx, ty, t1, t2, u1, u2, z;
// 
//   /* Calculate the number of points */
//   nn = 1;
// 
//   for(i = 0; i < m; i++)
//     nn *= 2;
// 
//   /* Do the bit reversal */
//   i2 = nn >> 1;
//   j = 0;
// 
//   for(i = 0; i < nn - 1; i++)
//   {
//     if(i < j)
//     {
//       tx = x[i];
//       ty = y[i];
//       x[i] = x[j];
//       y[i] = y[j];
//       x[j] = tx;
//       y[j] = ty;
//     }
// 
//     k = i2;
// 
//     while(k <= j)
//     {
//       j -= k;
//       k >>= 1;
//     }
// 
//     j += k;
//   }
// 
//   /* Compute the FFT */
//   c1 = -1.0;
//   c2 = 0.0;
//   l2 = 1;
// 
//   for(l = 0; l < m; l++)
//   {
//     l1 = l2;
//     l2 <<= 1;
//     u1 = 1.0;
//     u2 = 0.0;
// 
//     for(j = 0; j < l1; j++)
//     {
//       for(i = j; i < nn; i += l2)
//       {
//         i1 = i + l1;
//         t1 = u1 * x[i1] - u2 * y[i1];
//         t2 = u1 * y[i1] + u2 * x[i1];
//         x[i1] = x[i] - t1;
//         y[i1] = y[i] - t2;
//         x[i] += t1;
//         y[i] += t2;
//       }
// 
//       z =  u1 * c1 - u2 * c2;
//       u2 = u1 * c2 + u2 * c1;
//       u1 = z;
//     }
// 
//     c2 = sqrt((1.0 - c1) / 2.0);
// 
//     if(dir == 1)
//       c2 = -c2;
// 
//     c1 = sqrt((1.0 + c1) / 2.0);
//   }
// 
//   /* Scaling for forward transform */
//   if(dir == 1)
//   {
//     for(i = 0; i < nn; i++)
//     {
//       x[i] /= (double)nn;
//       y[i] /= (double)nn;
//     }
//   }
// 
//   return true;
// }
// 
// int Powerof2(int n, int *m, int *twopm)
// {
//   if(n <= 1)
//   {
//     *m = 0;
//     *twopm = 1;
//     return false;
//   }
// 
//   *m = 1;
//   *twopm = 2;
// 
//   do
//   {
//     (*m)++;
//     (*twopm) *= 2;
//   }
//   while(2 * (*twopm) <= n);
// 
//   if(*twopm != n)
//     return false;
//   else
//     return true;
// }

bool DFT::DFT2D(Complex2D &data, Direction dir)
{
  if(data.size() == 0 || data[0].size() == 0)
  {
    logger(LOG_ERROR) << "DFT: Number of rows is zero in input data?" << std::endl;
    return false;
  }
  
  _expTable.clear();
  
  SizeType rows = data.size(), columns = data[0].size();
  
  _internal.resize(rows);
  
  for(auto i = 0; i < rows; i++)
  {
    _internal[i].resize(columns);
    
    if(!DFT1D(data[i], _internal[i], dir))
    {
      logger(LOG_ERROR) << "Failed to perform 1D DFT for row '" << i << "'" << std::endl;
      return false;
    }
  }
  
  Complex1D in, out;
  in.resize(rows);
  out.resize(rows);
  
  for(auto i = 0; i < columns; i++)
  {
    for(auto j = 0; j < rows; j++)
      in[j] = _internal[j][i];
    
    if(!DFT1D(in, out, dir))
    {
      logger(LOG_ERROR) << "Failed to perform 1D DFT for row '" << i << "'" << std::endl;
      return false;
    }
    
    for(auto j = 0; j < rows; j++)
      data[j][i] = out[j];
  }
  
  return true;
}

bool DFT::DFT1D(const Complex1D &in, Complex1D &out, Direction dir)
{
  if(out.size() != in.size())
    out.resize(in.size());
  
  if(_expTable.find(in.size()) == _expTable.end())
  {
    _computeExpTable(in.size(), dir);
  }
  
  Complex2D &expTable = _expTable[in.size()];

  int i, k;
  
  int m = in.size();

  for(i = 0; i < m; i++)
  {
    out[i] = 0;
    for(k = 0; k < m; k++)
      out[i] += in[k]*expTable[i][k];
//     {
//       cosarg = cos(k*arg);
//       sinarg = sin(k*arg);
//       out[i].real(out[i].real() + (in[k].real() * cosarg - in[k].imag() * sinarg));
//       out[i].imag(out[i].imag() + (in[k].real() * sinarg + in[k].imag() * cosarg));
//     }
  }
  
  double msq = sqrt(m);
  for(i = 0; i < m; i++)
    out[i] /= msq;
  return true;
}

void DFT::_computeExpTable(SizeType size, Direction dir)
{
  Complex2D &expTable = _expTable[size];
  
  expTable.resize(size);
  for(int i = 0; i < size; i++)
  {
    expTable[i].resize(size);
    ComplexDouble arg = ComplexDouble(0, -((dir*2.0*M_PI*i)/size));
    
    for(int k = 0; k < size; k++)
      expTable[i][k] = exp(((double)k)*arg);
  }
}



}
