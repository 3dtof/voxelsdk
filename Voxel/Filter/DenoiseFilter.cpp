/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DenoiseFilter.h"
#if defined(ARM_OPT) || defined(x86_OPT)
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

#if defined(ARM_OPT) || defined(x86_OPT)
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
#if defined(ARM_OPT) || defined(x86_OPT)
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
#else
  _ampHistory.clear();
  _ambHistory.clear();
  _phaseHistory.clear();
  _flagsHistory.clear();
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
  __m128i sat = _mm_set1_epi16(8);
  unsigned short table[] = { 0x0001, 0x4000, 0x2000, 0x1555/*0x2AAB*/, 0, 0, 0, 0 };
  __m128i amp_sum, phase_sum, max_phase, min_phase, ones, valid, amp_out, valid_count, zeros, valid_ones;
  __m128i flags16, amp_curr, avg, amp_avg, valid1;
  __m128i xor_ones = _mm_set1_epi16(0xffff);
  int bits = 0;  
  
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
  
  for (int p = 0; p < s; p += 8)
  {
    amp_sum = _mm_set1_epi16(0);
    phase_sum = _mm_set1_epi16(0);
    max_phase = _mm_set1_epi16(0);
    min_phase = _mm_set1_epi16(0);
    ones = _mm_set1_epi16(1);
    zeros = _mm_set1_epi16(0);
    valid = _mm_set1_epi16(0);
    amp_out = _mm_set1_epi16(0);
    valid_count = _mm_set1_epi16(0);
    for (auto i = 0; i < cnt; i++)
    {
      /*--- LOADING THE VALUES OF FLAG AND STORING IT IN 16 BIT REGISTER ---*/
      flags16 = _mm_loadl_epi64((__m128i*)((unsigned char *)_flagsHistory[i] + bits));
      flags16 = _mm_unpacklo_epi8(flags16, zeros);
      flags16 = _mm_and_si128(flags16, sat);

      /*--- CHECKING THE SATURATION CONDITON AND INCREAMENTING THE VALID COUNT VARIABLE ---*/
      valid1 = _mm_cmpeq_epi16(flags16, sat);
      valid = _mm_xor_si128(valid1, xor_ones);
      valid_ones = _mm_and_si128(valid, ones);
      valid_count = _mm_add_epi16(valid_count, valid_ones);

      /*--- LOADING THE AMPLITUDE VALUE AND FINDING THE AMP SUM ---*/
      amp_curr = _mm_loadu_si128((__m128i*)((unsigned short *)_ampHistory[i] + bits));
      amp_sum = _mm_adds_epu16(amp_sum, _mm_and_si128(amp_curr, valid)); //saturating add check
    }

    {
      uint16_t ival = _mm_extract_epi16(valid_count, 0);
      avg = _mm_insert_epi16(avg, table[ival], 0);
      ival = _mm_extract_epi16(valid_count, 1);
      avg = _mm_insert_epi16(avg, table[ival], 1);
      ival = _mm_extract_epi16(valid_count, 2);
      avg = _mm_insert_epi16(avg, table[ival], 2);
      ival = _mm_extract_epi16(valid_count, 3);
      avg = _mm_insert_epi16(avg, table[ival], 3);
      ival = _mm_extract_epi16(valid_count, 4);
      avg = _mm_insert_epi16(avg, table[ival], 4);
      ival = _mm_extract_epi16(valid_count, 5);
      avg = _mm_insert_epi16(avg, table[ival], 5);
      ival = _mm_extract_epi16(valid_count, 6);
      avg = _mm_insert_epi16(avg, table[ival], 6);
      ival = _mm_extract_epi16(valid_count, 7);
      avg = _mm_insert_epi16(avg, table[ival], 7);
    }

    amp_avg = _mm_mulhi_epi16(amp_sum, avg);
    amp_avg = _mm_slli_epi16(amp_avg, 1);
    __m128i amp_avg1 = _mm_srli_epi16(_mm_mullo_epi16(amp_sum, avg), 15);
    amp_avg = _mm_or_si128(amp_avg, amp_avg1);
    amp_avg = _mm_slli_epi16(amp_avg, 1);

    valid = _mm_xor_si128(valid, xor_ones);
    amp_out = _mm_and_si128(valid, amp_curr);
    amp_out = _mm_or_si128(amp_out, amp_avg);

    /*--- STORING THE RESULT ---*/
    _mm_storeu_si128((__m128i*)((unsigned short *)ampOut + bits), amp_out);
    bits = bits + 8;
  }
  memcpy(phaseOut, phaseIn, phase_word_width);
  
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
