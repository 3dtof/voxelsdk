/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "HDRFilter.h"

#ifdef ARM_OPT
#include <arm_neon.h>
#endif

namespace Voxel
{
  
HDRFilter::HDRFilter(uint order): Filter("HDRFilter"), _order(order)
{
  _addParameters({
     FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", _order, "", 2, 100)),
  });
}

void HDRFilter::_onSet(const FilterParameterPtr &f)
{
  if (f->name() == "order")
  {
     if(!_get(f->name(), _order))
     {
        logger(LOG_WARNING) << "HDRFilter:  Could not get the recently updated 'order' parameter" << std::endl;
     }
  }
}

void HDRFilter::reset()
{
  _ampHistory.clear();
  _ambHistory.clear();
  _phaseHistory.clear();
  _flagsHistory.clear();
}

template <typename T>
bool HDRFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  for(auto i = 0; i < s; i++) 
    out[i] = in[i];
  
  return true;
}

#define SATURATED	(0x08)

template <typename PhaseT, typename AmpT>
bool HDRFilter::_filter2(const FramePtr &in_p, FramePtr &out_p)
{
   tVector<ByteType> buf;

   ToFRawFrame *in = dynamic_cast<ToFRawFrame *>(in_p.get());
   ToFRawFrame *out = dynamic_cast<ToFRawFrame *>(out_p.get());

   uint s = _size.width*_size.height;
 
   memcpy(out->ambient(), in->ambient(), s*in->ambientWordWidth());
   memcpy(out->flags(), in->flags(), s*in->flagsWordWidth());

#if 1
   buf.resize(s*in->amplitudeWordWidth());
   memcpy(buf.data(), in->amplitude(), s*in->amplitudeWordWidth());
   _ampHistory.push_back(std::move(buf));

   buf.resize(s*in->phaseWordWidth());
   memcpy(buf.data(), in->phase(), s*in->phaseWordWidth());
   _phaseHistory.push_back(std::move(buf));

   buf.resize(s*in->ambientWordWidth());
   memcpy(buf.data(), in->ambient(), s*in->ambientWordWidth());
   _ambHistory.push_back(std::move(buf));

   buf.resize(s*in->flagsWordWidth());
   memcpy(buf.data(), in->flags(), s*in->flagsWordWidth());
   _flagsHistory.push_back(std::move(buf));
#endif

   if (_ampHistory.size() > _order) 
   {
     _ampHistory.pop_front();
     _phaseHistory.pop_front();
     _ambHistory.pop_front();
     _flagsHistory.pop_front();
   }

   AmpT *ampIn = (AmpT *)in->amplitude();
   AmpT *ampOut = (AmpT *)out->amplitude();
   PhaseT *phaseIn = (PhaseT *)in->phase();
   PhaseT *phaseOut = (PhaseT *)out->phase();

#ifdef x86_OPT

  __m128i vSat = _mm_set1_epi16(0x0008);
  __m128i vOnes = _mm_set1_epi16(0xFFFF);
  __m128i vZeros = _mm_set1_epi16(0);
  
  AmpT *pAmpIn = ampIn;
  AmpT *pAmpOut = ampOut;
  PhaseT *pPhaseIn = phaseIn;
  PhaseT *pPhaseOut = phaseOut;
  
  for (int p = 0; p < s; p+=8)
  {
    __m128i vMax_i = _mm_set1_epi16(0xFFFF);
    __m128i vMax_Amp = vZeros;
    __m128i vMax_Phase = vZeros;
    __m128i vAmpOut, vPhaseOut, vAmpIn, vPhaseIn;
    
    vAmpIn = _mm_loadu_si128((__m128i*)pAmpIn);
    vPhaseIn = _mm_loadu_si128((__m128i*)pPhaseIn);
    
    for (auto i = 0; i < _ampHistory.size(); i++)
    {
      AmpT *amp_cur = (AmpT *)_ampHistory[i].data();
      PhaseT *phase_cur = (PhaseT *)_phaseHistory[i].data();
      uint8_t *flags_cur = (uint8_t *)_flagsHistory[i].data();
      __m128i vi = _mm_set1_epi16((uint16_t)i);
      
      __m128i vFlags_cur8 = _mm_loadl_epi64((__m128i*)(flags_cur + p));
      __m128i vAmp_cur = _mm_loadu_si128((__m128i*)(amp_cur + p));
      __m128i vPhase_cur = _mm_loadu_si128((__m128i*)(phase_cur + p));
      __m128i vFlags_cur = _mm_unpacklo_epi8(vFlags_cur8, vZeros);
      __m128i vCondition1, vCondition2, vCondition, vConditionInv;
      
      vCondition1 = _mm_cmpeq_epi16(_mm_and_si128(vFlags_cur, vSat), vSat);
      vCondition1 = _mm_xor_si128(vCondition1, vOnes);
      vCondition2 = _mm_cmpgt_epi16(vAmp_cur, vMax_Amp);
  
      vCondition = _mm_and_si128(vCondition1, vCondition2);
      vConditionInv =  _mm_xor_si128(vCondition, vOnes);
  
      vMax_i = _mm_or_si128(_mm_and_si128(vi, vCondition), _mm_and_si128(vMax_i, vConditionInv));
      vMax_Amp = _mm_or_si128(_mm_and_si128(vAmp_cur, vCondition), _mm_and_si128(vMax_Amp, vConditionInv));
      vMax_Phase = _mm_or_si128(_mm_and_si128(vPhase_cur, vCondition), _mm_and_si128(vMax_Phase, vConditionInv));
      
    }
    
    __m128i valid; 
    __m128i inValid;
    
    inValid = _mm_cmpeq_epi16(vMax_i, vOnes);
    valid = _mm_xor_si128(inValid, vOnes);
    
    vAmpOut = _mm_or_si128(_mm_and_si128(vMax_Amp, valid), _mm_and_si128(vAmpIn, inValid));
    vPhaseOut = _mm_or_si128(_mm_and_si128(vMax_Phase, valid), _mm_and_si128(vPhaseIn, inValid));
    
    _mm_storeu_si128((__m128i*)pAmpOut, vAmpOut);
    _mm_storeu_si128((__m128i*)pPhaseOut, vPhaseOut);
    
    pAmpIn += 8;
    pAmpOut += 8;
    pPhaseIn += 8;
    pPhaseOut += 8;
  }
  
#elif ARM_OPT
    uint16x8_t vSat = vdupq_n_u16(0x0008);
    uint16x8_t vOnes = vdupq_n_u16(0xFFFF);
    uint16x8_t vZeros = vdupq_n_u16(0);
    
    AmpT *pAmpIn = ampIn;
    AmpT *pAmpOut = ampOut;
    PhaseT *pPhaseIn = phaseIn;
    PhaseT *pPhaseOut = phaseOut;
    
    for (int p = 0; p < s; p+=8)
    {
      uint16x8_t vMax_i = vdupq_n_u16(0xFFFF);
      uint16x8_t vMax_Amp = vZeros;
      uint16x8_t vMax_Phase = vZeros;
      uint16x8_t vAmpOut, vPhaseOut, vAmpIn, vPhaseIn;
      
      vAmpIn = vld1q_u16((uint16_t*)pAmpIn);
      vPhaseIn = vld1q_u16((uint16_t*)pPhaseIn);
      
      for (auto i = 0; i < _ampHistory.size(); i++)
      {
        AmpT *amp_cur = (AmpT *)_ampHistory[i].data();
        PhaseT *phase_cur = (PhaseT *)_phaseHistory[i].data();
        uint8_t *flags_cur = (uint8_t *)_flagsHistory[i].data();
        uint16x8_t vi = vdupq_n_u16((uint16_t)i);
        
        uint8x8_t vFlags_cur8 = vld1_u8((uint8_t*)flags_cur + p);
        uint16x8_t vAmp_cur = vld1q_u16((uint16_t*)amp_cur + p);
        uint16x8_t vPhase_cur = vld1q_u16((uint16_t*)phase_cur + p);
        uint16x8_t vFlags_cur = vmovl_u8(vFlags_cur8);
        uint16x8_t vCondition1, vCondition2, vCondition, vConditionInv;
        
        vCondition1 = vceqq_u16(vandq_u16(vFlags_cur, vSat), vSat);
        vCondition1 = vmvnq_u16(vCondition1);
        vCondition2 = vcgtq_u16(vAmp_cur, vMax_Amp);

        vCondition = vandq_u16(vCondition1, vCondition2);
        vConditionInv =  vmvnq_u16(vCondition);

        vMax_i = vorrq_u16(vandq_u16(vi, vCondition), vandq_u16(vMax_i, vConditionInv));
        vMax_Amp = vorrq_u16(vandq_u16(vAmp_cur, vCondition), vandq_u16(vMax_Amp, vConditionInv));
        vMax_Phase = vorrq_u16(vandq_u16(vPhase_cur, vCondition), vandq_u16(vMax_Phase, vConditionInv));
        
      }
      
      uint16x8_t valid; 
      uint16x8_t inValid;
      
      inValid = vceqq_u16(vMax_i, vOnes);
      valid = vmvnq_u16(inValid);
      
      vAmpOut = vorrq_u16(vandq_u16(vMax_Amp, valid), vandq_u16(vAmpIn, inValid));
      vPhaseOut = vorrq_u16(vandq_u16(vMax_Phase, valid), vandq_u16(vPhaseIn, inValid));
      
      vst1q_u16((uint16_t*)pAmpOut, vAmpOut);
      vst1q_u16((uint16_t*)pPhaseOut, vPhaseOut);
      
      pAmpIn += 8;
      pAmpOut += 8;
      pPhaseIn += 8;
      pPhaseOut += 8;
    }
    
#else
   for (int p = 0; p < s; p++) 
   {
      int max_i = -1;
      AmpT max_amp = 0;
      PhaseT max_phase = 0;

      for (auto i = 0; i < _ampHistory.size(); i++)
      {
#if 1
         AmpT *amp_cur = (AmpT *)_ampHistory[i].data();
         PhaseT *phase_cur = (PhaseT *)_phaseHistory[i].data();
         uint8_t *flags_cur = (uint8_t *)_flagsHistory[i].data();
#else
         AmpT *amp_cur = (AmpT *)in->amplitude();
         PhaseT *phase_cur = (PhaseT *)in->phase();
         uint8_t *flags_cur = (uint8_t *)in->flags();
#endif
         if ( (flags_cur[p] & SATURATED) != SATURATED ) 
         {
            if (amp_cur[p] > max_amp) 
            {
               max_i = i;  
               max_amp = amp_cur[p];
               max_phase = phase_cur[p];
            }
         }
      }
      
      if (max_i != -1) 
      {
         ampOut[p] = max_amp;
         phaseOut[p] = max_phase;
      }
      else 
      {
         ampOut[p] = ampIn[p];
         phaseOut[p] = phaseIn[p];
      }
   }

#endif
   return true;
}

bool HDRFilter::_filter(const FramePtr &in, FramePtr &out)
{
   bool ret;
   ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
   DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
   if ((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
   {
      logger(LOG_ERROR) << "HDRFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
      return false;
   }
  
   if (tofFrame)
   {
      _size = tofFrame->size;
      ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
      if (!o)
      {
         logger(LOG_ERROR) << "HDRFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
         return false;
      }
    
      if (tofFrame->phaseWordWidth() == 2)
      {
         if (tofFrame->amplitudeWordWidth() == 1) 
            ret = _filter2<uint16_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            ret = _filter2<uint16_t, uint16_t>(in, out);
         else if(tofFrame->amplitudeWordWidth() == 4)
            ret = _filter2<uint16_t, uint32_t>(in, out);
      }
      else if (tofFrame->phaseWordWidth() == 1)
      {
         if (tofFrame->amplitudeWordWidth() == 1)
            ret = _filter2<uint8_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            ret = _filter2<uint8_t, uint16_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 4)
            ret = _filter2<uint8_t, uint32_t>(in, out);
      }
      else if (tofFrame->phaseWordWidth() == 4)
      {
         if (tofFrame->amplitudeWordWidth() == 1)
            ret = _filter2<uint32_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            ret = _filter2<uint32_t, uint16_t>(in, out);
         else if(tofFrame->amplitudeWordWidth() == 4)
            ret = _filter2<uint32_t, uint32_t>(in, out);
      }
   }
   else if(depthFrame)
   {
      _size = depthFrame->size;
      DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
      if (!o)
      {
         logger(LOG_ERROR) << "HDRFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
         return false;
      }
    
      o->amplitude = depthFrame->amplitude;
    
      ret = _filter<float>(depthFrame->depth.data(), o->depth.data());
   }
   return ret;
}
  
}
