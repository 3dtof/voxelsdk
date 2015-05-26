/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFCrossTalkFilter.h"

#include <iterator>

namespace Voxel
{

ToFCrossTalkFilter::ToFCrossTalkFilter(): Filter("ToFCrossTalkFilter")
{
}

bool ToFCrossTalkFilter::_filter(const FramePtr &in, FramePtr &out)
{
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  
  if(_coefficients.size() != 9)
    return false;
  
  if(!tofFrame || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "ToFCrossTalkFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "ToFCrossTalkFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());
    
    if(tofFrame->phaseWordWidth() == 2)
      return _filter<uint16_t>((uint16_t *)tofFrame->amplitude(), (uint16_t *)tofFrame->phase(), (uint16_t *)o->amplitude(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      return _filter<uint8_t>((uint8_t *)tofFrame->amplitude(), (uint8_t *)tofFrame->phase(), (uint8_t *)o->amplitude(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      return _filter<uint32_t>((uint32_t *)tofFrame->amplitude(), (uint32_t *)tofFrame->phase(), (uint32_t *)o->amplitude(), (uint32_t *)o->phase());
    else
      return false;
  }
}

bool ToFCrossTalkFilter::setMaxPhaseRange(uint32_t maxPhaseRange)
{
  _maxPhaseRange = maxPhaseRange;
  _phaseToAngleFactor =  2*M_PI/_maxPhaseRange;
  _angleToPhaseFactor = _maxPhaseRange/(2*M_PI); 
  return true;
}

bool ToFCrossTalkFilter::readCoefficients(const String &coefficients)
{
  _coefficients.reserve(9);
  
  std::istringstream ss(coefficients);
  
  std::copy(std::istream_iterator<Complex>(ss),
            std::istream_iterator<Complex>(),
            std::back_inserter(_coefficients));
  
  if(_coefficients.size() != 9)
    return false;
  
  // Normalize the plus sign filter
  float magnitude = std::abs(_coefficients[1]) + 
    std::abs(_coefficients[3]) + std::abs(_coefficients[4]) + std::abs(_coefficients[5]) +
    std::abs(_coefficients[7]);
    
  _coefficients[1] /= magnitude;
  _coefficients[3] /= magnitude;
  _coefficients[4] /= magnitude;
  _coefficients[5] /= magnitude;
  _coefficients[7] /= magnitude;
  return true;
}


#define ToF_CROSS_TALK_PROCESS_RESULT \
*ao++ = abs(result); \
phase = std::arg(result);\
*po++ = ((phase < 0)?(2*M_PI + phase):phase)*_angleToPhaseFactor;\
d++;

template <typename T>
bool ToFCrossTalkFilter::_filter(const T *amplitudeIn, const T *phaseIn, T *amplitudeOut, T *phaseOut)
{
  if(_coefficients.size() != 9)
    return false;
  
  uint s = _size.width*_size.height;
  _amplitudePhase.resize(s);
  
  T *ao = amplitudeOut, *po = phaseOut;
  const T *ai = amplitudeIn, *pi = phaseIn;
  
  int i = s, j;
  
  Complex *d = _amplitudePhase.data();
  
  while(i--)
    *d++ = std::polar<float>(*ai++, *pi++*_phaseToAngleFactor);
  
  d = _amplitudePhase.data();
  
  Complex result;
  float phase;
  
  // Top-left corner
  result = (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5] + (*(d + _size.width))*_coefficients[7];
  ToF_CROSS_TALK_PROCESS_RESULT
  
  // Top row
  i = _size.width - 2;
  while(i--)
  {
    result = (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5] + (*(d + _size.width))*_coefficients[7];
    ToF_CROSS_TALK_PROCESS_RESULT
  }
  
  // Top-right corner
  result = (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4] + (*(d + _size.width))*_coefficients[7];
  ToF_CROSS_TALK_PROCESS_RESULT
  
  
  // All the way till bottom row (excluding)
  j = _size.height - 2;
  
  #pragma omp parallel for
  for(j = 1; j < _size.height - 1; j++)
  {
    int i;
    Complex result; // local variables for parallel looping
    float phase;
    int offset = _size.width*j;
    Complex *d = _amplitudePhase.data() + offset;
    T *ao = amplitudeOut + offset, *po = phaseOut + offset;
    
    // left corner
    result =  (*(d - _size.width))*_coefficients[1] + (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5] + (*(d + _size.width))*_coefficients[7];
    ToF_CROSS_TALK_PROCESS_RESULT
    
    // row
    i = _size.width - 2;
    while(i--)
    {
      result = (*(d - _size.width))*_coefficients[1] + (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5] + (*(d + _size.width))*_coefficients[7];
      ToF_CROSS_TALK_PROCESS_RESULT
    }
    
    // right corner
    result = (*(d - _size.width))*_coefficients[1] + (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4] + (*(d + _size.width))*_coefficients[7];
    ToF_CROSS_TALK_PROCESS_RESULT
  }
  
  int offset = _size.width*(_size.height - 1);
  ao = amplitudeOut + offset;
  po = phaseOut + offset;
  
  // bottom-left corner
  result =  (*(d - _size.width))*_coefficients[1] + (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5];
  ToF_CROSS_TALK_PROCESS_RESULT
  
  // bottom row
  i = _size.width - 2;
  while(i--)
  {
    result = (*(d - _size.width))*_coefficients[1] + (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4] + (*(d + 1))*_coefficients[5];
    ToF_CROSS_TALK_PROCESS_RESULT
  }
  
  // bottom-right corner
  result = (*(d - _size.width))*_coefficients[1] + (*(d - 1))*_coefficients[3] + (*d)*_coefficients[4];
  ToF_CROSS_TALK_PROCESS_RESULT
  
  /*
  for(auto j = 0; j < _size.height; j++)
    for(auto i = 0; i < _size.width; i++)
      if(phaseIn[i + j*_size.width] != phaseOut[i + j*_size.width])
      {
        std::cout << "(" << i << ", " << j << ") = " << phaseIn[i + j*_size.width] << " -> " << phaseOut[i + j*_size.width] 
        << _amplitudePhase[i + j*_size.width] << std::endl;
      }
  */
  
  return true;
}


void ToFCrossTalkFilter::reset() 
{
  _amplitudePhase.clear();
}


  
}