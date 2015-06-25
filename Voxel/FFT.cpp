/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 * 
 * Borrowed from http://paulbourke.net/miscellaneous/dft/
 * 
 * Reference: DFT (Discrete Fourier Transform) and FFT (Fast Fourier Transform) 
 * Written by Paul Bourke, June 1993
 * 
 */

#include <malloc.h>
#include "FFT.h"

#include "Logger.h"

namespace Voxel
{

  

bool FFT2D(Complex2D &c,int dir)
{
  if(c.size() == 0)
    return false;
  
  int i,j;
  int m,twopm;
  double *real,*imag;
  
  int nx = c[0].size();
  int ny = c.size();
  
  /* Transform the rows */
  real = (double *)malloc(nx * sizeof(double));
  imag = (double *)malloc(nx * sizeof(double));
  if (real == NULL || imag == NULL)
  {
    logger(LOG_ERROR) << "FFT2D: Failed to allocate memory" << std::endl;
    return false;
  }
  
  if (!Powerof2(nx,&m,&twopm) || twopm != nx)
  {
    logger(LOG_ERROR) << "FFT2D: Failed to get nearest power of 2 of '" << nx << "'" << std::endl;
    return false;
  }
  
  for (j=0;j<ny;j++) {
    for (i=0;i<nx;i++) {
      real[i] = c[i][j].real();
      imag[i] = c[i][j].imag();
    }
    FFT(dir,m,real,imag);
    for (i=0;i<nx;i++) {
      c[i][j].real(real[i]);
      c[i][j].imag(imag[i]);
    }
  }
  free(real);
  free(imag);
  
  /* Transform the columns */
  real = (double *)malloc(ny * sizeof(double));
  imag = (double *)malloc(ny * sizeof(double));
  if (real == NULL || imag == NULL)
  {
    logger(LOG_ERROR) << "FFT2D: Failed to allocate memory" << std::endl;
    return false;
  }
  
  if (!Powerof2(ny,&m,&twopm) || twopm != ny)
  {
    logger(LOG_ERROR) << "FFT2D: Failed to get nearest power of 2 of '" << nx << "'" << std::endl;
    return false;
  }
  
  for (i=0;i<nx;i++) {
    for (j=0;j<ny;j++) {
      real[j] = c[i][j].real();
      imag[j] = c[i][j].imag();
    }
    FFT(dir,m,real,imag);
    for (j=0;j<ny;j++) {
      c[i][j].real(real[j]);
      c[i][j].imag(imag[j]);
    }
  }
  free(real);
  free(imag);
  
  return true;
}

bool FFT(int dir,int m,double *x,double *y)
{
  long nn,i,i1,j,k,i2,l,l1,l2;
  double c1,c2,tx,ty,t1,t2,u1,u2,z;
  
  /* Calculate the number of points */
  nn = 1;
  for (i=0;i<m;i++)
    nn *= 2;
  
  /* Do the bit reversal */
  i2 = nn >> 1;
  j = 0;
  for (i=0;i<nn-1;i++) {
    if (i < j) {
      tx = x[i];
      ty = y[i];
      x[i] = x[j];
      y[i] = y[j];
      x[j] = tx;
      y[j] = ty;
    }
    k = i2;
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }
  
  /* Compute the FFT */
  c1 = -1.0;
  c2 = 0.0;
  l2 = 1;
  for (l=0;l<m;l++) {
    l1 = l2;
    l2 <<= 1;
    u1 = 1.0;
    u2 = 0.0;
    for (j=0;j<l1;j++) {
      for (i=j;i<nn;i+=l2) {
        i1 = i + l1;
        t1 = u1 * x[i1] - u2 * y[i1];
        t2 = u1 * y[i1] + u2 * x[i1];
        x[i1] = x[i] - t1;
        y[i1] = y[i] - t2;
        x[i] += t1;
        y[i] += t2;
      }
      z =  u1 * c1 - u2 * c2;
      u2 = u1 * c2 + u2 * c1;
      u1 = z;
    }
    c2 = sqrt((1.0 - c1) / 2.0);
    if (dir == 1)
      c2 = -c2;
    c1 = sqrt((1.0 + c1) / 2.0);
  }
  
  /* Scaling for forward transform */
  if (dir == 1) {
    for (i=0;i<nn;i++) {
      x[i] /= (double)nn;
      y[i] /= (double)nn;
    }
  }
  
  return true;
}

int Powerof2(int n,int *m,int *twopm)
{
  if (n <= 1) {
    *m = 0;
    *twopm = 1;
    return false;
  }
  
  *m = 1;
  *twopm = 2;
  do {
    (*m)++;
    (*twopm) *= 2;
  } while (2*(*twopm) <= n);
  
  if (*twopm != n)
    return false;
  else
    return true;
}


}