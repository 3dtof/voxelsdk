/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "MedianFilter.h"

#include <float.h>

namespace Voxel
{

MedianFilter::MedianFilter(float stability, float deadband, float deadbandStep, uint halfKernelSize): Filter("MedianFilter"), 
  _stability(stability), _deadband(deadband), _deadbandStep(deadbandStep), _halfKernelSize(halfKernelSize)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("stability", "Stability", "Stability factor", stability, "", 0, 1)),
    FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", deadband, "", 0, 1)),
    FilterParameterPtr(new FloatFilterParameter("deadbandStep", "Dead band step", "Dead band step", deadbandStep, "", 0, 1)),
    FilterParameterPtr(new UnsignedFilterParameter("halfKernelSize", "Half kernel size", "Half kernel size", halfKernelSize, "", 1, 100))
  });
}

void MedianFilter::reset() { _current.clear(); }

void MedianFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "stability")
  {
    if(!_get(f->name(), _stability))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'stability' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadband")
  {
    if(!_get(f->name(), _deadband))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadbandStep")
  {
    if(!_get(f->name(), _deadbandStep))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'deadbandStep' parameter" << std::endl;
    }
  }
  else if(f->name() == "halfKernelSize")
  {
    if(!_get(f->name(), _halfKernelSize))
    {
      logger(LOG_WARNING) << "MedianFilter: Could not get the recently updated 'halfKernelSize' parameter" << std::endl;
    }
  }
}

template <typename T>
bool MedianFilter::_filter(const T *in, T *out)
{
#if defined(ARM_OPT) || defined (x86_OPT)

  uint s = _size.width*_size.height;

  T *cur, *hist;

  unsigned int transSize =   s*sizeof(T);
  if(_current.size() != transSize)
  {
    _current.resize(transSize);
    memset(_current.data(), 0, transSize);
  }
  
  uint histSize = (2*_halfKernelSize + 1)*(2*_halfKernelSize + 1) * sizeof(T);
  
  if(_hist.size() != histSize)
  {
    _hist.resize(histSize);
    memset(_hist.data(), 0, histSize);
  }
  
  cur = (T *)_current.data();
  hist = (T *)_hist.data();
  
  int stablePixel = _size.width*_size.height;
  
  int index;
  int order = 2 * _halfKernelSize + 1;
  int nWidthMinus1 = _size.width - 1;
  int nHeightMinus1 = _size.height - 1;

  if(order == 3)
  {
#ifdef x86_OPT

    /*---------------------------------------------------------------*/
    /*             TO FIND THE MEDIAN OF CENTRE PIXELS              */
    /*---------------------------------------------------------------*/

    __m128i vec0, vec1, vec2, vec3, vec4, vec5, vec6, vec7, vec8;
    uint16_t *Temp_Array = (uint16_t *)malloc(16);
    T *out1 = out;
    T *cur1 = cur;
    out1 += _size.width+1;
    cur1 += _size.width+1;
    const T *in1=in;
    
    /* dead = _deadband * 2 ^ 18 + 1*/
    float dead = _deadband*262144+1;
    __m128i vdead = _mm_set1_epi16((uint)dead);
    __m128i temp_mul;
    __m128i result;
    __m128i result1, result2;
    __m128i temp_cmp;

    __m128i temp;
    __m128i vstability = _mm_set1_epi16(stablePixel);
  
    __m128i ones_1 = _mm_set1_epi16(1);
    __m128i zeros = _mm_set1_epi16(0);
    __m128i ones = _mm_set1_epi16(0xffff);
      
    for(int32_t vindex = 0; vindex < _size.height - 2; vindex++)
    {
      __m128i row1_a = _mm_loadu_si128((__m128i*)(uint16_t *)in1);                      //load first 16 values from row 1
      __m128i row1_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + 8));                 //load second 16 values from row1
      __m128i row2_a = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + _size.width));              //load first 16 values from row 2
      __m128i row2_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + _size.width + 8));          //load second 16 values from row 2
      __m128i row3_a = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + (_size.width << 1)));       //load first 16 values from row 3
      __m128i row3_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + (_size.width << 1) + 8));  //load second 16 values from row 3
      int count = 0;
      
      for(int32_t hindex = 0; hindex < _size.width; hindex += 8)
      {
        __m128i diff, negDiff, absDiff;
        __m128i ones_1 = _mm_set1_epi16(0x01);
        __m128i vcur = _mm_loadu_si128((__m128i*)(uint16_t *)cur1);
        
        vec0 = row1_a;
        vec1 = row2_a;
        vec2 = row3_a;
  
        vec3 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + 1));
        vec4 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + _size.width + 1));
        vec5 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + (_size.width << 1) + 1));
  
        vec6 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + 2));
        vec7 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + _size.width + 2));
        vec8 = _mm_loadu_si128((__m128i*)((uint16_t*)in1 + (_size.width << 1) + 2));
  
  
        /*---- Paeth's Algorithm Starts here----*/
        vmin_max(vec0, vec3);
        vmin_max(vec1, vec4);
  
        vmin_max(vec0, vec1);
        vmin_max(vec2, vec5);
  
        vmin_max(vec0, vec2);
        vmin_max(vec4, vec5);
  
        vmin_max(vec1, vec2);
        vmin_max(vec3, vec5);
  
        vmin_max(vec3, vec4);
        vmin_max(vec1, vec3);
  
        vmin_max(vec1, vec6);
        vmin_max(vec4, vec6);
  
        vmin_max(vec2, vec6);
        vmin_max(vec2, vec3);
  
        vmin_max(vec4, vec7);
        vmin_max(vec2, vec4);
  
        vmin_max(vec3, vec7);
        vmin_max(vec4, vec8);
  
        vmin_max(vec3, vec8);
        vmin_max(vec3, vec4);   
        /*--- vec4 contains median of corresponding to 9 pixels ---*/
        
        row1_a = row1_b;
        row2_a = row2_b;
        row3_a = row3_b;
        in1 = in1 + 8;
        row1_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + 8));
        row2_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + _size.width + 8));
        row3_b = _mm_loadu_si128((__m128i*)((uint16_t *)in1 + (_size.width << 1) + 8));
        
        diff = _mm_sub_epi16(vec4, vcur);
        negDiff = _mm_sub_epi16(zeros, diff);
        absDiff = _mm_max_epi16(diff, negDiff);
        
        temp_mul = _mm_mulhi_epu16(vdead, vcur);
        temp_mul = _mm_srli_epi16(temp_mul, 2);
        
        temp_cmp = _mm_cmpgt_epi16(absDiff, temp_mul);
        ones_1 = _mm_and_si128(temp_cmp, ones_1);
        stablePixel -= _mm_extract_epi16(ones_1, 0) +
            _mm_extract_epi16(ones_1, 1) +
            _mm_extract_epi16(ones_1, 2) +
            _mm_extract_epi16(ones_1, 3) +
            _mm_extract_epi16(ones_1, 4) +
            _mm_extract_epi16(ones_1, 5) +
            _mm_extract_epi16(ones_1, 6) +
            _mm_extract_epi16(ones_1, 7);
  
        result1 = _mm_and_si128(temp_cmp, vec4);
        result2 = _mm_xor_si128(temp_cmp, ones);
        result2 = _mm_and_si128(result2, vcur);
        result = _mm_or_si128(result1, result2);
        
        _mm_storeu_si128((__m128i*)(uint16_t *)out1, result);
        _mm_storeu_si128((__m128i*)(uint16_t *)cur1, result);
        
        out1 += 8;
        cur1 += 8; 
        
      }
    }

#elif ARM_OPT
    /*---------------------------------------------------------------*/
    /*             TO FIND THE MEDIAN OF CENTRE PIXELS              */
    /*---------------------------------------------------------------*/

    uint16x8_t vec0, vec1, vec2, vec3, vec4, vec5, vec6, vec7, vec8;
    uint16_t *Temp_Array = (uint16_t *)malloc(16);  
    T *out1 = out;
    T *cur1 = cur;
    out1 += _size.width+1;
    cur1 += _size.width+1;
    const T *in1=in;
    
    int nFlag = 1;
    
    /* dead = _deadband * 2 ^ 18 + 1*/
    float dead = _deadband*262144+1 ;
    uint16x8_t vdead = vdupq_n_u16((uint)dead);
    uint16x8_t temp_mul;
    uint16x8_t temp_str;
    uint16x8_t temp_str1;
    uint16x8_t temp_cmp;
    uint16x8_t subq;
    uint16x8_t temp;
    uint16x8_t vstability = vdupq_n_u16(stablePixel); 
    uint16x8_t ones_1 = vdupq_n_u16(1);
    
    for(int32_t vindex = 0; vindex < _size.height - 2; vindex++)
    {
      uint16x8_t row1_a = vld1q_u16((uint16_t *)in1);                      //load first 16 values from row 1
      uint16x8_t row1_b = vld1q_u16((uint16_t *)in1 + 8);                 //load second 16 values from row1
      uint16x8_t row2_a = vld1q_u16((uint16_t *)in1 + _size.width);              //load first 16 values from row 2
      uint16x8_t row2_b = vld1q_u16((uint16_t *)in1 + _size.width + 8);          //load second 16 values from row 2
      uint16x8_t row3_a = vld1q_u16((uint16_t *)in1 + (_size.width << 1));       //load first 16 values from row 3
      uint16x8_t row3_b = vld1q_u16((uint16_t *)in1 + (_size.width << 1) + 8);  //load second 16 values from row 3
      int count = 0;
      
      for(int32_t hindex = 0; hindex < _size.width; hindex += 8)
      {
        uint16x8_t ones_1 = vdupq_n_u16(0x01);  
        uint16x8_t vcur = vld1q_u16((uint16_t *)cur1);
        
        vec0 = row1_a;
        vec1 = row2_a;
        vec2 = row3_a;
  
        vec3 = vextq_u16(row1_a, row1_b, 1);
        vec4 = vextq_u16(row2_a, row2_b, 1);
        vec5 = vextq_u16(row3_a, row3_b, 1);
  
        vec6 = vextq_u16(row1_a, row1_b, 2);
        vec7 = vextq_u16(row2_a, row2_b, 2);
        vec8 = vextq_u16(row3_a, row3_b, 2);
  
  
        /*---- Paeth's Algorithm Starts here----*/
        vmin_max(vec0, vec3);
        vmin_max(vec1, vec4);
  
        vmin_max(vec0, vec1);
        vmin_max(vec2, vec5);
  
        vmin_max(vec0, vec2);
        vmin_max(vec4, vec5);
  
        vmin_max(vec1, vec2);
        vmin_max(vec3, vec5);
  
        vmin_max(vec3, vec4);
        vmin_max(vec1, vec3);
  
        vmin_max(vec1, vec6);
        vmin_max(vec4, vec6);
  
        vmin_max(vec2, vec6);
        vmin_max(vec2, vec3);
  
        vmin_max(vec4, vec7);
        vmin_max(vec2, vec4);
  
        vmin_max(vec3, vec7);
        vmin_max(vec4, vec8);
  
        vmin_max(vec3, vec8);
        vmin_max(vec3, vec4);   
        /*--- vec4 contains median of corresponding to 9 pixels ---*/
        
        row1_a = row1_b;
        row2_a = row2_b;
        row3_a = row3_b;
        in1 = in1 + 8;
        row1_b = vld1q_u16((uint16_t *)in1 + 8);
        row2_b = vld1q_u16((uint16_t *)in1 + _size.width + 8);
        row3_b = vld1q_u16((uint16_t *)in1 + (_size.width << 1) + 8);
  
        subq = vsubq_u16(vec4,vcur);
        temp = vreinterpretq_u16_s16(vabsq_s16(vreinterpretq_s16_u16(subq)));
        
        temp_mul = (vreinterpretq_u16_s16(vqdmulhq_s16(vreinterpretq_s16_u16(vdead),vreinterpretq_s16_u16(vcur)))); //MULTIPLYING REFERENCE PIXEL AND DEADBAND VALUE  
        temp_mul = vshrq_n_u16(temp_mul,3); 
        
        temp_cmp = vcgtq_u16(temp,temp_mul);
        ones_1 = vandq_u16(temp_cmp,ones_1);
        stablePixel -= ones_1[0]+ones_1[1]+ones_1[2]+ones_1[3]+ones_1[4]+ones_1[5]+ones_1[6]+ones_1[7];
        
        temp_str = vandq_u16(temp_cmp, vec4);
        
        temp_str1 = vandq_u16(vmvnq_u16(temp_cmp), vcur);
  
        temp_str = vorrq_u16(temp_str, temp_str1);
        
        
        vst1q_u16((uint16_t *)out1  , temp_str);
        vst1q_u16((uint16_t *)cur1  , temp_str);  
        
        out1 += 8;
        cur1 += 8; 
        
      
      }
    
    }
#endif
  
    /*----------------------------------------------------------------*/
    /*        TO FIND THE MEDIAN OF FIRST AND LAST COLUMN             */
    /*----------------------------------------------------------------*/

    for (int j = 1; j < nHeightMinus1; j++) 
    {
      for (int i = 0; i < _size.width; i+=nWidthMinus1) 
      {
        int p = j*_size.width + i;
        index = 0;
        
        /*** Loading pixels to calculate median ***/
        for (int k = -(int)_halfKernelSize; k <= (int)_halfKernelSize; k++) 
        {
          for (int m = -(int)_halfKernelSize; m <= (int)_halfKernelSize; m++) 
          {
            int i2 = i+m; int j2 = j+k;
            if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
            {
              int q = j2*_size.width+i2;
              hist[index++] = in[q];
            }
          }
        }

        int s, temp = 0, r;
        T hist_temp;
        int size_inb2 = index/2;
        for(r = 0; r <= size_inb2; r++)
        {
          for(s = r+1, temp = r; s < index; s++)
          {
            if(hist[s] < hist[temp])
              temp = s;
          }
          if(temp != r)
          {
            hist_temp = hist[temp];
            hist[temp]= hist[r];
          }
        }
        T val;
        if(temp == (r-1))
          val = hist[temp];
        else
          val = hist_temp;
        
        /* Output w/ deadband */
        float ferr = cur[p]?fabs((float)(val- cur[p])/cur[p]):FLT_MAX;
        
        if (ferr > _deadband) 
        {
          out[p] = cur[p] = val;
          stablePixel--;
        }
        else
          out[p] = cur[p];
      }  // for (i)
    } // for (j)

    /*-------------------------------------------------------------*/
    /*            TO FIND THE MEDIAN OF FIRST AND LAST ROW         */
    /*-------------------------------------------------------------*/

    /*** Loading pixels to calculate median ***/
    for (int j = 0; j < _size.height; j+=nHeightMinus1) 
    {
      for (int i = 0; i < _size.width; i++) 
      {
        int p = j*_size.width + i;
        index = 0;
        
        for (int k =-(int)_halfKernelSize; k <= (int)_halfKernelSize; k++) 
        {
          for (int m = -(int)_halfKernelSize; m <= (int)_halfKernelSize; m++) 
          {
            int i2 = i+m; int j2 = j+k;
            if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
            {
              int q = j2*_size.width+i2;
              hist[index++] = in[q];
            }
          }
        }

        int s, temp = 0, r;
        T hist_temp;
        int size_inb2 = index/2;
        for(r = 0; r <= size_inb2; r++)
        {
          for(s = r+1, temp = r; s < index; s++)
          {
            if(hist[s] < hist[temp])
              temp = s;
          }
          if(temp != r)
          {
            hist_temp = hist[temp];
            hist[temp]= hist[r];
          }
        }
        T val;
        if(temp == (r-1))
          val = hist[temp];
        else
          val = hist_temp;
        
        /* Output w/ deadband */
        float ferr = cur[p]?fabs((float)(val- cur[p])/cur[p]):FLT_MAX;
        
        if (ferr > _deadband) 
        {
          out[p] = cur[p] = val;
          stablePixel--;
        }
        else
          out[p] = cur[p];
      }  // for (i)
    } // for (j)

  }
  else
  {
    for (int j = 0; j < _size.height; j++) 
    {
      for (int i = 0; i < _size.width; i++) 
      {
        int p = j*_size.width + i;
        index = 0;

        for (int k = -(int)_halfKernelSize; k <= (int)_halfKernelSize; k++) 
        {
          for (int m = -(int)_halfKernelSize; m <= (int)_halfKernelSize; m++) 
          {
            int i2 = i+m; int j2 = j+k;
            if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
            {
              int q = j2*_size.width+i2;
              hist[index++] = in[q];
            }
          }
        }

        std::nth_element(hist, hist + index/2, hist + index);
        T val = hist[index/2];

        // Output w/ deadband
        float ferr = cur[p]?fabs((float)(val- cur[p])/cur[p]):FLT_MAX;

        if (ferr > _deadband) 
        {
          out[p] = cur[p] = val;
          stablePixel--;
        }
        else
          out[p] = cur[p];
      }  // for (i)
    } // for (j)
  }
  /* Adjust deadband until ratio is achieved */
  float diff = (float)stablePixel - _stability*_size.width*_size.height;
  if (diff < 0)
    _deadband += _deadbandStep;
  else
    _deadband -= _deadbandStep;

  _set("deadband", _deadband);

#else
  uint s = _size.width*_size.height;
  
  T *cur, *hist;
  
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  uint histSize = (2*_halfKernelSize + 1)*(2*_halfKernelSize + 1);
  
  if(_hist.size() != histSize*sizeof(T))
  {
    _hist.resize(histSize*sizeof(T));
    memset(_hist.data(), 0, histSize*sizeof(T));
  }
  
  cur = (T *)_current.data();
  hist = (T *)_hist.data();
  
  int stablePixel = _size.width*_size.height;
  
  int index;
  
  for (int j = 0; j < _size.height; j++) 
  {
    for (int i = 0; i < _size.width; i++) 
    {
      int p = j*_size.width + i;
      index = 0;
      
      for (int k = -(int)_halfKernelSize; k <= (int)_halfKernelSize; k++) 
      {
        for (int m = -(int)_halfKernelSize; m <= (int)_halfKernelSize; m++) 
        {
          int i2 = i+m; int j2 = j+k;
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) 
          {
            int q = j2*_size.width+i2;
            hist[index++] = in[q];
          }
        }
      }
      std::nth_element(hist, hist + index/2, hist + index);
      T val = hist[index/2];
      
      // Output w/ deadband
      float ferr = cur[p]?fabs((float)(val- cur[p])/cur[p]):FLT_MAX;
      
      if (ferr > _deadband) 
      {
        out[p] = cur[p] = val;
        stablePixel--;
      }
      else
        out[p] = cur[p];
    }  // for (i)
  } // for (j)
  
  // Adjust deadband until ratio is achieved
  float diff = (float)stablePixel - _stability*_size.width*_size.height;
  if (diff < 0)
    _deadband += _deadbandStep;
  else
    _deadband -= _deadbandStep;
  
  _set("deadband", _deadband);
  
#endif
  return true;
}


bool MedianFilter::_filter(const FramePtr &in, FramePtr &out)
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
    /*** amplitudeWordWidth and phaseWordWidth are same ***/
    /*** ambientWordWidth and flagsWordWidth are same ***/
    
    unsigned int size1 = s*tofFrame->ambientWordWidth();
    unsigned int size2 = s*tofFrame->amplitudeWordWidth();

    memcpy(o->ambient(), tofFrame->ambient(), size1);
    memcpy(o->amplitude(), tofFrame->amplitude(), size2);
    memcpy(o->flags(), tofFrame->flags(), size1);
    
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