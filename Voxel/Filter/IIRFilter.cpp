/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "IIRFilter.h"

#ifdef ARM_OPT
#include <arm_neon.h>
#endif

namespace Voxel
{
  
IIRFilter::IIRFilter(float gain): Filter("IIRFilter"), _gain(gain)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("gain", "Gain", "IIR gain coefficient", gain, "", 0.0f, 1.0f))
  });
}

void IIRFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "gain")
    if(!_get(f->name(), _gain))
    {
      logger(LOG_WARNING) << "IIRFilter:  Could not get the recently updated 'gain' parameter" << std::endl;
    }
}

void IIRFilter::reset()
{
  _current.clear();
}

template <typename T>
bool IIRFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  T *cur;
  
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  cur = (T *)_current.data();

#ifdef ARM_OPT

  if(_gain == 0.5)
  {
    uint16x8_t vIn, vOut, vCur;
    const T *pIn = in;
    T *pOut = out;
    T *pCur = cur;

    const int16_t nGain = _gain*2;
    for(auto i = 0; i < s; i+=8)
    {
      vIn = vld1q_u16((uint16_t*)pIn);
      vCur = vld1q_u16((uint16_t*)pCur);
      
      vOut = vhaddq_u16(vIn, vCur);
      
      vst1q_u16((uint16_t*)pOut, vOut);
      vst1q_u16((uint16_t*)pCur, vOut);
      
      pIn += 8;
      pOut += 8;
      pCur += 8;
    }
  }

#elif x86_OPT

  if(_gain == 0.5)
  {
    __m128i vIn, vOut, vCur;
    const T *pIn = in;
    T *pOut = out;
    T *pCur = cur;

    const int16_t nGain = _gain*2;
    for(auto i = 0; i < s; i+=8)
    {
      vIn = _mm_loadu_si128((__m128i*)pIn);
      vCur = _mm_loadu_si128((__m128i*)pCur);
      
      vOut = _mm_srli_epi16(_mm_add_epi16(vIn, vCur), 1);
      
      _mm_storeu_si128((__m128i*)pOut, vOut);
      _mm_storeu_si128((__m128i*)pCur, vOut);
      
      pIn += 8;
      pOut += 8;
      pCur += 8;
    }
  }
#else 
  for(auto i = 0; i < s; i++) 
    out[i] = cur[i] = cur[i]*(1.0 - _gain) + in[i]*_gain;
  
#endif
  return true;
}

bool IIRFilter::_filter(const FramePtr &in, FramePtr &out)
{
  bool ret;
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "IIRFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    //logger(LOG_INFO) << "IIRFilter: Applying filter with gain = " << _gain << " to ToFRawFrame id = " << tofFrame->id << std::endl;
    
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());
    
    if(tofFrame->phaseWordWidth() == 2)
      ret = _filter<uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      ret = _filter<uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      ret = _filter<uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)o->phase());
    else
      return false;
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "IIRFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    ret = _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  return ret;
}
  
}