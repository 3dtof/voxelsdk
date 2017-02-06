/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DarkPixFilter.h"

#ifdef ARM_OPT
#include <arm_neon.h>
#endif

namespace Voxel
{
  
DarkPixFilter::DarkPixFilter(float aThrNear, float phThrNear, float aThrFar, float phThrFar, float ambThresh): Filter("DarkPixFilter"), _aThrNear(aThrNear), _phThrNear(phThrNear), _aThrFar(aThrFar), _phThrFar(phThrFar), _ambThresh(ambThresh)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("aThrNear", "aThrNear", "NearAmpThr", _aThrNear, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("phThrNear", "phThrNear", "NearPhThr", _phThrNear, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("aThrFar", "aThrFar", "FarAmpThr", _aThrFar, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("phThrFar", "phThrFar", "FarPhThr", _phThrFar, "", 0.0f, 4095.0f)),
    FilterParameterPtr(new FloatFilterParameter("ambThresh", "ambThresh", "Ambient Threshold", _ambThresh, "", 0.0f, 15.0f))
  });
}

void DarkPixFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "aThrNear")
  {
    if(!_get(f->name(), _aThrNear))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'aThrNear' parameter" << std::endl;
    }
  }
  else if(f->name() == "phThrNear") 
  {
    if(!_get(f->name(), _phThrNear))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'phThrNear' parameter" << std::endl;
    }
  }
  else if(f->name() == "aThrFar")
  {
    if(!_get(f->name(), _aThrFar))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'aThrFar' parameter" << std::endl;
    }
  }
  else if(f->name() == "phThrFar") 
  {
    if(!_get(f->name(), _phThrFar))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'phThrFar' parameter" << std::endl;
    }
  }
  else if(f->name() == "ambThresh") 
  {
    if(!_get(f->name(), _ambThresh))
    {
      logger(LOG_WARNING) << "DarkPixFilter:  Could not get the recently updated 'ambThresh' parameter" << std::endl;
    }
  }
}

void DarkPixFilter::reset()
{
  _current.clear();
}

template <typename T>
bool DarkPixFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  for(auto i = 0; i < s; i++) 
    out[i] = in[i];
  
  return true;
}

template <typename T, typename T2>
bool DarkPixFilter::_filter2(const T *in, T2 *amp, uint8_t *amb, T *out)
{
  uint s = _size.width*_size.height;
#ifdef x86_OPT

  T2 *pAmp = amp;
  uint8_t *pAmb = amb;
  const T *pIn = in;
  T *pOut = out;

  __m128i vIn, vOut, vAmp, vAmb;
  __m128i vAmb8;
  __m128i vAThrNear = _mm_set1_epi16((uint16_t)_aThrNear);
  __m128i vPhThrNear = _mm_set1_epi16((uint16_t)_phThrNear);
  __m128i vAThrFar = _mm_set1_epi16((uint16_t)_aThrFar);
  __m128i vPhThrFar = _mm_set1_epi16((uint16_t)_phThrFar);
  __m128i vAmbThresh = _mm_set1_epi16((uint16_t)_ambThresh);
  __m128i vZeros = _mm_set1_epi16(0);
  __m128i xor_ones = _mm_set1_epi16(0xffff);

  for (auto p = 0; p < s; p += 8)
  {
    vIn = _mm_loadu_si128((__m128i*)pIn);
    vAmp = _mm_loadu_si128((__m128i*)pAmp);
    vAmb8 = _mm_loadl_epi64((__m128i*)pAmb);
    vAmb = _mm_unpacklo_epi8(vAmb8, vZeros);
    __m128i vC1, vC2, vC3, vValid, vInValid;

    vC1 = _mm_and_si128(_mm_cmplt_epi16(vAmp, vAThrNear), _mm_cmplt_epi16(vIn, vPhThrNear));
    vC2 = _mm_and_si128(_mm_cmpgt_epi16(vAmp, vAThrFar), _mm_cmpgt_epi16(vIn, vPhThrFar));
    vC3 = _mm_cmplt_epi16(vAmb, vAmbThresh);

    vValid = _mm_or_si128(_mm_or_si128(vC1, vC2), vC3);
    vInValid = _mm_xor_si128(vValid, xor_ones);

    vOut = _mm_or_si128(_mm_and_si128(vValid, vZeros), _mm_and_si128(vInValid, vIn));
    _mm_storeu_si128((__m128i*)pOut, vOut);

    pAmp += 8;
    pAmb += 8;
    pIn += 8;
    pOut += 8;
  }
  
#elif ARM_OPT
  T2 *pAmp = amp;
  uint8_t *pAmb = amb;
  const T *pIn = in;
  T *pOut = out;
  
  uint16x8_t vIn, vOut, vAmp, vAmb;
  uint8x8_t vAmb8;
  uint16x8_t vAThrNear = vdupq_n_u16((uint16_t)_aThrNear);
  uint16x8_t vPhThrNear = vdupq_n_u16((uint16_t)_phThrNear);
  uint16x8_t vAThrFar = vdupq_n_u16((uint16_t)_aThrFar);
  uint16x8_t vPhThrFar = vdupq_n_u16((uint16_t)_phThrFar);
  uint16x8_t vAmbThresh = vdupq_n_u16((uint16_t)_ambThresh);
  uint16x8_t vZeros = vdupq_n_u16(0);
  
  for (auto p = 0; p < s; p+=8)
  {
    vIn = vld1q_u16((uint16_t*)pIn);
    vAmp = vld1q_u16((uint16_t*)pAmp);
    vAmb8 = vld1_u8((uint8_t*)pAmb);
    vAmb = vmovl_u8(vAmb8);
    uint16x8_t vC1, vC2, vC3, vValid, vInValid;
    
    vC1 = vandq_u16(vcltq_u16(vAmp, vAThrNear), vcltq_u16(vIn, vPhThrNear));
    vC2 = vandq_u16(vcgtq_u16(vAmp, vAThrFar), vcgtq_u16(vIn, vPhThrFar));
    vC3 = vcltq_u16(vAmb, vAmbThresh);
    
    vValid = vorrq_u16(vorrq_u16(vC1, vC2), vC3);
    vInValid = vmvnq_u16(vValid);
    
    vOut = vorrq_u16(vandq_u16(vValid, vZeros), vandq_u16(vInValid, vIn));
    vst1q_u16((uint16_t*)pOut, vOut);
    
    pAmp += 8;
    pAmb += 8;
    pIn += 8;
    pOut += 8;
  }

#else
  for (auto p = 0; p < s; p++)
  {
    if (((amp[p] < _aThrNear) && (in[p] < _phThrNear)) || ((amp[p] > _aThrFar) && (in[p] > _phThrFar)) || ((amb[p] < _ambThresh)))
      out[p] = 0;
    else
      out[p] = in[p];
  }
#endif
  return true;
}

bool DarkPixFilter::_filter(const FramePtr &in, FramePtr &out)
{
  bool ret;
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "DarkPixFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "DarkPixFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
    //logger(LOG_INFO) << "DarkPixFilter: Applying filter with aThr = " << _aThr << " and phThr = " << _phThr << std::endl;
    
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    //memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());

    if(tofFrame->phaseWordWidth() == 2)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        ret = _filter2<uint16_t, uint8_t>((uint16_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), o->ambient(), (uint16_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2) {
        _filter2<uint16_t, uint16_t>((uint16_t *)tofFrame->amplitude(), (uint16_t *)tofFrame->amplitude(), o->ambient(), (uint16_t *)o->amplitude());
        ret = _filter2<uint16_t, uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), o->ambient(), (uint16_t *)o->phase());
      }
      else if(tofFrame->amplitudeWordWidth() == 4)
        ret = _filter2<uint16_t, uint32_t>((uint16_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), o->ambient(), (uint16_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 1)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        ret = _filter2<uint8_t, uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), o->ambient(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2)
        ret = _filter2<uint8_t, uint16_t>((uint8_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), o->ambient(), (uint8_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 4)
        ret = _filter2<uint8_t, uint32_t>((uint8_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), o->ambient(), (uint8_t *)o->phase());
    }
    else if(tofFrame->phaseWordWidth() == 4)
    {
      if(tofFrame->amplitudeWordWidth() == 1)
        ret = _filter2<uint32_t, uint8_t>((uint32_t *)tofFrame->phase(), (uint8_t *)tofFrame->amplitude(), o->ambient(), (uint32_t *)o->phase());
      else if(tofFrame->amplitudeWordWidth() == 2) {
        ret = _filter2<uint32_t, uint16_t>((uint32_t *)tofFrame->phase(), (uint16_t *)tofFrame->amplitude(), o->ambient(), (uint32_t *)o->phase());
      }
      else if(tofFrame->amplitudeWordWidth() == 4)
        ret = _filter2<uint32_t, uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)tofFrame->amplitude(), o->ambient(), (uint32_t *)o->phase());
    }
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "DarkPixFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    ret = _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  return ret;
}
  
}
