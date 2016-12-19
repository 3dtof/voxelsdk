/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DenoiseFilter.h"

namespace Voxel
{
  
DenoiseFilter::DenoiseFilter(uint order, float threshold): Filter("DenoiseFilter"), _order(order), _threshold(threshold)
{
  _addParameters({
     FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", _order, "", 2, 100)),
     FilterParameterPtr(new FloatFilterParameter("threshold", "threshold", "Threshold", _threshold, "", 0, 10000)),
  });
}

void DenoiseFilter::_onSet(const FilterParameterPtr &f)
{
  if (f->name() == "order")
  {
     if(!_get(f->name(), _order))
     {
        logger(LOG_WARNING) << "DenoiseFilter:  Could not get the recently updated 'order' parameter" << std::endl;
     }
  }
  else if (f->name() == "threshold")
  {
     if(!_get(f->name(), _threshold))
     {
        logger(LOG_WARNING) << "DenoiseFilter:  Could not get the recently updated 'threshold' parameter" << std::endl;
     }
  }
}

void DenoiseFilter::reset()
{
  _ampHistory.clear();
  _ambHistory.clear();
  _phaseHistory.clear();
  _flagsHistory.clear();
}

template <typename T>
bool DenoiseFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  for(auto i = 0; i < s; i++) 
    out[i] = in[i];
  
  return true;
}

#define SATURATED	(0x08)

template <typename PhaseT, typename AmpT>
bool DenoiseFilter::_filter2(const FramePtr &in_p, FramePtr &out_p)
{
   Vector<ByteType> buf;

   ToFRawFrame *in = dynamic_cast<ToFRawFrame *>(in_p.get());
   ToFRawFrame *out = dynamic_cast<ToFRawFrame *>(out_p.get());

   uint s = _size.width*_size.height;
 
#ifdef x86_OPT
   /*** amplitudeWordWidth and phaseWordWidth are same ***/
   /*** ambientWordWidth and flagsWordWidth are same ***/
     
   unsigned int size1 = s*in->amplitudeWordWidth();   
   unsigned int size2 = s*in->ambientWordWidth();   
   
   memcpy(out->ambient(), in->ambient(), size2);
   memcpy(out->flags(), in->flags(), size2);

   buf.resize(size1);
   memcpy(buf.data(), in->amplitude(), size1);
   _ampHistory.push_back(std::move(buf));

   buf.resize(size1);
   memcpy(buf.data(), in->phase(), size1);
   _phaseHistory.push_back(std::move(buf));

   buf.resize(size2);
   memcpy(buf.data(), in->ambient(), size2);
   _ambHistory.push_back(std::move(buf));

   buf.resize(size2);
   memcpy(buf.data(), in->flags(), size2);
   _flagsHistory.push_back(std::move(buf));

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
   /*** loop unroll by 4 ***/
   
   for (int p = 0; p < s; p+=4) 
   {
      bool valid1 = false;
      bool valid2 = false;
      bool valid3 = false;
      bool valid4 = false;
      uint valid_cnt1 = 0;
      uint valid_cnt2 = 0;
      uint valid_cnt3 = 0;
      uint valid_cnt4 = 0;
      float amp_sum1 = 0, phase_sum1 = 0;
      float amp_sum2 = 0, phase_sum2 = 0;
      float amp_sum3 = 0, phase_sum3= 0;
      float amp_sum4 = 0, phase_sum4 = 0;

      for (auto i = 0; i < _ampHistory.size(); i++)
      {
         AmpT *amp_cur = (AmpT *)_ampHistory[i].data();
         PhaseT *phase_cur = (PhaseT *)_phaseHistory[i].data();
         uint8_t *flags_cur = (uint8_t *)_flagsHistory[i].data();

         if ( (flags_cur[p] & SATURATED) != SATURATED ) 
         {
            valid1 = true; valid_cnt1++;
            amp_sum1 += (float)amp_cur[p]; 
         }
         if ( (flags_cur[p+1] & SATURATED) != SATURATED ) 
         {
            valid2 = true; valid_cnt2++;
            amp_sum2 += (float)amp_cur[p+1]; 
         }
         if ( (flags_cur[p+2] & SATURATED) != SATURATED ) 
         {
            valid3 = true; valid_cnt3++;
            amp_sum3 += (float)amp_cur[p+2]; 
         }
         if ( (flags_cur[p+3] & SATURATED) != SATURATED ) 
         {
            valid4 = true; valid_cnt4++;
            amp_sum4 += (float)amp_cur[p+3]; 
         }
      }
      
      float amp_avg1   = amp_sum1/valid_cnt1;
      float amp_avg2   = amp_sum2/valid_cnt2;
      float amp_avg3   = amp_sum3/valid_cnt3;
      float amp_avg4   = amp_sum4/valid_cnt4;

      if (valid1) 
         ampOut[p] = amp_avg1; 
      else 
         ampOut[p] = ampIn[p];

      if (valid2) 
         ampOut[p+1] = amp_avg2;
      else 
         ampOut[p+1] = ampIn[p+1];

      if (valid3) 
         ampOut[p+2] = amp_avg3;
      else 
         ampOut[p+2] = ampIn[p+2];

      if (valid4) 
         ampOut[p+3] = amp_avg4;
      else 
         ampOut[p+3] = ampIn[p+3];
   }
   memcpy(phaseOut, phaseIn, size1);
#else
   memcpy(out->ambient(), in->ambient(), s*in->ambientWordWidth());
   memcpy(out->flags(), in->flags(), s*in->flagsWordWidth());

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

   for (int p = 0; p < s; p++) 
   {
      bool valid = false;
      uint valid_cnt = 0;
      float amp_sum = 0, phase_sum = 0;
      PhaseT max_phase = 0, min_phase = 0;

      for (auto i = 0; i < _ampHistory.size(); i++)
      {
         AmpT *amp_cur = (AmpT *)_ampHistory[i].data();
         PhaseT *phase_cur = (PhaseT *)_phaseHistory[i].data();
         uint8_t *flags_cur = (uint8_t *)_flagsHistory[i].data();

         if ( (flags_cur[p] & SATURATED) != SATURATED ) 
         {
            valid = true; valid_cnt++;
            max_phase = (phase_cur[p] > max_phase) ? phase_cur[p] : max_phase;
            min_phase = (phase_cur[p] < min_phase) ? phase_cur[p] : min_phase;
            phase_sum += (float)phase_cur[p];
            amp_sum += (float)amp_cur[p]; 
         }
      }
      
      float amp_avg = amp_sum/valid_cnt;
      float phase_avg = phase_sum/valid_cnt;

      if (valid) 
      {
         float var1 = (float)(max_phase - min_phase);
         float var2 = _threshold * amp_avg;
         ampOut[p] = (var1 <= var2) ? (AmpT)amp_avg : (AmpT)0;
         //phaseOut[p] = (var1 <= var2) ? (PhaseT)phase_avg : (PhaseT)0;
         phaseOut[p] = phaseIn[p];
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

bool DenoiseFilter::_filter(const FramePtr &in, FramePtr &out)
{
   ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
   DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
   if ((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
   {
      logger(LOG_ERROR) << "DenoiseFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
      return false;
   }
  
   if (tofFrame)
   {
      _size = tofFrame->size;
      ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
      if (!o)
      {
         logger(LOG_ERROR) << "DenoiseFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
         return false;
      }
    
      if (tofFrame->phaseWordWidth() == 2)
      {
         if (tofFrame->amplitudeWordWidth() == 1)
            return _filter2<uint16_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            return _filter2<uint16_t, uint16_t>(in, out);
         else if(tofFrame->amplitudeWordWidth() == 4)
            return _filter2<uint16_t, uint32_t>(in, out);
      }
      else if (tofFrame->phaseWordWidth() == 1)
      {
         if (tofFrame->amplitudeWordWidth() == 1)
            return _filter2<uint8_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            return _filter2<uint8_t, uint16_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 4)
            return _filter2<uint8_t, uint32_t>(in, out);
      }
      else if (tofFrame->phaseWordWidth() == 4)
      {
         if (tofFrame->amplitudeWordWidth() == 1)
            return _filter2<uint32_t, uint8_t>(in, out);
         else if (tofFrame->amplitudeWordWidth() == 2)
            return _filter2<uint32_t, uint16_t>(in, out);
         else if(tofFrame->amplitudeWordWidth() == 4)
            return _filter2<uint32_t, uint32_t>(in, out);
      }
   }
   else if(depthFrame)
   {
      _size = depthFrame->size;
      DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
      if (!o)
      {
         logger(LOG_ERROR) << "DenoiseFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
         return false;
      }
    
      o->amplitude = depthFrame->amplitude;
    
      return _filter<float>(depthFrame->depth.data(), o->depth.data());
   }
   else
      return false;
}
  
}
