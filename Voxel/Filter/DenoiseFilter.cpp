/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DenoiseFilter.h"
#ifdef ARM_OPT
extern int32_t nFrameWidth, nFrameHeight;
#endif

namespace Voxel
{
  
DenoiseFilter::DenoiseFilter(uint order, float threshold): Filter("DenoiseFilter"), _order(order), _threshold(threshold)
{
  _addParameters({
     FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", _order, "", 2, 100)),
     FilterParameterPtr(new FloatFilterParameter("threshold", "threshold", "Threshold", _threshold, "", 0, 10000)),
  });
#ifdef ARM_OPT
  _ampHistory = (ByteType **)malloc((_order+1)*sizeof(ByteType *));
  _phaseHistory = (ByteType **)malloc((_order+1)*sizeof(ByteType *));
  _ambHistory = (ByteType **)malloc((_order+1)*sizeof(ByteType *));
  _flagsHistory = (ByteType **)malloc((_order+1)*sizeof(ByteType *));
  
  denoise_frames = 0;
  cnt = 0;
  
  _ampHistory[0] = NULL;
  _ampHistory[1] = NULL;
  _ampHistory[2] = NULL;
  
  _phaseHistory[0] = NULL;
  _phaseHistory[1] = NULL;
  _phaseHistory[2] = NULL;
  
  _ambHistory[0] = NULL;
  _ambHistory[1] = NULL;
  _ambHistory[2] = NULL;
  
  _flagsHistory[0] = NULL;
  _flagsHistory[1] = NULL;
  _flagsHistory[2] = NULL;
  

#endif
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
#ifndef ARM_OPT 
  _ampHistory.clear();
  _ambHistory.clear();
  _phaseHistory.clear();
  _flagsHistory.clear();
#else
  for(int i = 0; i < 3; i++)
  {
    if(_ampHistory[i] != NULL)
      free(_ampHistory[i]);
    if(_ambHistory[i] != NULL)
      free(_ambHistory[i]);
    if(_phaseHistory[i] != NULL)
      free(_phaseHistory[i]);
    if(_flagsHistory[i] != NULL)
      free(_flagsHistory[i]);
  }
  _ampHistory[0]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _ampHistory[1]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _ampHistory[2]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);

  _phaseHistory[0]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _phaseHistory[1]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _phaseHistory[2]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);

  _ambHistory[0]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _ambHistory[1]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _ambHistory[2]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);

  _flagsHistory[0]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _flagsHistory[1]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _flagsHistory[2]=(ByteType *)malloc(nFrameWidth*nFrameHeight*4);
#endif
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
#elif ARM_OPT
  auto phase_word_width = s*in->phaseWordWidth();
  auto amp_word_width = phase_word_width;
  auto flags_word_width = s*in->flagsWordWidth();
  auto ambi_word_width = flags_word_width; 

  memcpy(out->ambient(), in->ambient(), ambi_word_width);
  memcpy(out->flags(), in->flags(), flags_word_width);

  memcpy(_ampHistory[denoise_frames], in->amplitude(), amp_word_width);
  memcpy(_phaseHistory[denoise_frames], in->phase(), phase_word_width);
  memcpy(_ambHistory[denoise_frames], in->ambient(), ambi_word_width); 
  memcpy(_flagsHistory[denoise_frames], in->flags(), flags_word_width);

  denoise_frames = (denoise_frames == (_order-1)) ? 0: ++denoise_frames;
  cnt++;
  cnt = (cnt < _order ? cnt : _order);

  AmpT *ampIn = (AmpT *)in->amplitude();
  AmpT *ampOut = (AmpT *)out->amplitude();
  PhaseT *phaseIn = (PhaseT *)in->phase();
  PhaseT *phaseOut = (PhaseT *)out->phase();

  uint16x8_t var1;
  uint16x8_t var2;
  uint16x8_t amp_out;
  uint8_t arr[] =  {1,00,00,85, 00, 205, 171, 37};
  uint8_t arr1[] = {1,64,32,21, 16, 12, 10, 9};
  uint8x8_t table1;
  uint8x8_t table2;
  uint8x8x2_t zip;
  uint16x8_t avg;
  uint16x8_t thres;
  uint8x16_t combine;
  uint16x8_t ones16;
  uint16x8_t ones_16;
  uint16x8_t mask_16;
  uint16x8_t amp_sum;
  uint16x8_t phase_sum;
  uint16x8_t amp_avg;
  uint16x8_t phase_avg;
  uint16x8_t valid_ones;
  uint8x8_t valid_count8;
  uint8x8_t valid_count8_1;
  uint8x8_t valid_count8_2;
  uint16x8_t phase_curr;
  uint16x8_t amp_curr;
  uint16x8_t valid;
  uint8x8_t flags8;
  uint16x8_t sat;
  uint16x8_t flags16;
  uint16x8_t ones;
  uint16x8_t valid_count;
  uint16x8_t max_phase;
  uint16x8_t min_phase;
  int bits = 0;

  mask_16 = vdupq_n_u16(32767);
  ones16 = vdupq_n_u16(32768);
  thres = vdupq_n_u16(_threshold);
  table1 = vld1_u8(arr);           //table1 contains the lsb of number of frames
  table2 = vld1_u8(arr1);          //table2 contains the msb
  s = s / 8;

  for (int p = 0; p < s; p++)
  {
    sat = vdupq_n_u16(8);
    amp_sum = vdupq_n_u16(0);
    phase_sum = vdupq_n_u16(0);
    max_phase = vdupq_n_u16(0);
    min_phase = vdupq_n_u16(0);
    ones = vdupq_n_u16(1);
    valid = vdupq_n_u16(0);
    amp_out = vdupq_n_u16(0);
    valid_count = vdupq_n_u16(0);

    for (auto i = 0; i < cnt; i++)
     {
       /*--- LOADING THE VALUES OF FLAG AND STORING IT IN 16 BIT REGISTER ---*/  
       flags8 = vld1_u8(((uint8_t *)_flagsHistory[i]) + bits);   
       flags16 = vmovl_u8(flags8);                                
       flags16 = vandq_u16(flags16, sat);
       /*--- CHECKING THE SATURATION CONDITON AND INCREAMENTING THE VALID COUNT VARIABLE ---*/
       valid = vceqq_u16(flags16, sat);
       valid = vmvnq_u16(valid);
       valid_ones = vandq_u16(valid, ones);
       valid_count = vaddq_u16(valid_count, valid_ones);
       /*--- LOADING THE AMPLITUDE VALUE AND FINDING THE AMP SUM ---*/ 
       amp_curr = vld1q_u16(((uint16_t *)_ampHistory[i]) + bits);
       amp_sum = vqaddq_u16(amp_sum, vandq_u16(amp_curr, valid));
     }
     
     /*--- DIVIDING THE AMPLITUDE SUM BY  VALID COUNT ---*/    
     valid_count8 = vmovn_u16(valid_count);
     
     /*--- LOADING VALUES FROM THE TABLE ---*/
     valid_count8_2 = vtbl1_u8(table2, valid_count8);               
     valid_count8_1 = vtbl1_u8(table1, valid_count8);
     
     /*--- ZIPPING TWO EIGHT BIT INTO ONE 16 BIT VARIABLE ---*/
     zip = vzip_u8(valid_count8_1, valid_count8_2);                 
     combine = vcombine_u8(zip.val[0], zip.val[1]);                 
     
     /*--- COMBINING THE ZIPPED VARIABLE ---*/
     avg = vreinterpretq_u16_u8(combine);
     
     /*--- DIVISION THROUGH MULTIPLICATION --*/
      /*--- VQDMULHQ WILL MULTIPLY TWO VECTORS(VECTOR COMPRISES OF EIGHT ELEMENT AND EACH ELEMENT SIZE IS 16-BIT) AND STORES THE RESULT IN 
            32-BIT VECTOR AND SHIFTS THE RESULT 16 TIMES TO RIGHT ---*/     
     amp_avg = (vreinterpretq_u16_s16(vqdmulhq_s16(vreinterpretq_s16_u16(amp_sum), vreinterpretq_s16_u16(avg))));   //multiplying without themsb
     amp_avg = vshlq_n_u16(amp_avg,1);
     valid = vmvnq_u16(valid);
     amp_out = vandq_u16(valid, amp_curr);
     amp_out = vorrq_u16(amp_out, amp_avg);
     
     /*--- STORING THE RESULT ---*/
     vst1q_u16(((uint16_t *)ampOut) + bits, amp_out);
     bits = bits + 8;
   }
   memcpy(phaseOut, phaseIn, phase_word_width);
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
  bool ret;
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
      logger(LOG_ERROR) << "DenoiseFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
   
    o->amplitude = depthFrame->amplitude;
   
    ret = _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  return ret;
}
  
}
