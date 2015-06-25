/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Convolve2D.h"
#include "Logger.h"

namespace Voxel
{

Convolve2D::Convolve2D(const Vector<float> &coefficients, SizeType rows, SizeType columns):_rows(rows), _columns(columns)
{
  if(coefficients.size() != _rows*_columns)
  {
    logger(LOG_ERROR) << "Convolve2D: Invalid number of coefficients given. Expecting " 
    << _rows << "x" << _columns << " number of coefficients." << std::endl;
    _rows = _columns = 0;
  }
  else if(_rows % 2 == 0 || _columns % 2 == 0)
  {
    logger(LOG_ERROR) << "Convolve2D: Rows and columns are expected to be odd numbers. Got " 
    << _rows << "x" << _columns << "." << std::endl;
    _rows = _columns = 0;
  }
  else
  {
    SizeType s = coefficients.size();
    _coefficients.resize(s);
    for(auto i = 0; i < s; i++)
      _coefficients[s - i - 1] = coefficients[i]; // Invert and keep ready for convolution
  }
}

#define COEFF(r, c) _coefficients[(rc + r)*_columns + (cc + c)]

bool Convolve2D::convolve(const Vector<float> &in, SizeType rows, SizeType columns, Vector<float> &out)
{
  if(_coefficients.size() == 0 || in.size() != rows*columns)
  {
    logger(LOG_ERROR) << "Convolve2D: Invalid data size or missing coefficients." << std::endl;
    return false;
  }
  
  out.resize(rows*columns);
  
  int rc = _rows/2, cc = _columns/2;
  
  for(int r = 0; r < rows; r++)
    for(int c = 0; c < columns; c++)
    {
      auto i = r*columns + c;
      
      float v = 0;
      
      for(int r1 = -rc; r1 <= rc; r1++)
        for(int c1 = -cc; c1 <= cc; c1++)
        {
          if(r + r1 < 0 || r + r1 >= rows || c + c1 < 0 || c + c1 >= columns)
            continue;
          
          auto j = r1*columns + c1;
          
          v += in[i + j]*COEFF(r1, c1);
          
        }
        
      out[i] = v;
    }
  
  return true;
}

  
  
}