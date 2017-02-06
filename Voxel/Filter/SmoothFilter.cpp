
#include "SmoothFilter.h"

#include <memory.h>

#ifdef ARM_OPT
#include <arm_neon.h>
#endif

#ifdef x86_OPT
static inline __m128i muly_32(const __m128i &a, const __m128i &b)
{
  __m128i tmp1 = _mm_mul_epu32(a, b); /* mul 2,0*/
  __m128i tmp2 = _mm_mul_epu32(_mm_srli_si128(a, 4), _mm_srli_si128(b, 4)); /* mul 3,1 */
  return _mm_unpacklo_epi32(_mm_shuffle_epi32(tmp1, _MM_SHUFFLE(0, 0, 2, 0)), _mm_shuffle_epi32(tmp2, _MM_SHUFFLE(0, 0, 2, 0))); /* shuffle results to [63..0] and pack */
}
#endif

namespace Voxel
{

SmoothFilter::SmoothFilter(float sigma): Filter("SmoothFilter"), _discreteGaussian(sigma)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("sigma", "Sigma", "Standard deviation", sigma, "", 0, 100))
  });
}

void SmoothFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "sigma")
  {
    float s;
    if(!_get(f->name(), s))
    {
      logger(LOG_WARNING) << "SmoothFilter: Could not get the recently updated 'sigma' parameter" << std::endl;
    }
    _discreteGaussian.setStandardDeviation(s);
  }
}

void SmoothFilter::reset() {}

template <typename T>
bool SmoothFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;

#ifdef x86_OPT

  int tempHeight = _size.height-2;
  int tempwidth = _size.width-2;

  if(_discreteGaussian.getStandardDeviation() == 0.5)
  {
    /*table contents = round(product * pow(2,16))*/
    uint16_t WeightTable[40] = {0x0000, 0x0002, 0x000E, 0x0002, 0x0000, 0x0000, 0x0000, 0x0000,\
                                0x0002, 0x02FC, 0x160E, 0x02FC, 0x0002, 0x0000, 0x0000, 0x0000,\
                                0x000E, 0x160E, 0xA2FA, 0x160E, 0x000E, 0x0000, 0x0000, 0x0000,\
                                0x0002, 0x02FC, 0x160E, 0x02FC, 0x0002, 0x0000, 0x0000, 0x0000,\
                                0x0000, 0x0002, 0x000E, 0x0002, 0x0000, 0x0000, 0x0000, 0x0000};
  
    __m128i vWeightTable;
    /* vWeight_sum = (1/weight_sum) << 16 */
    __m128i vWeight_sum = _mm_set1_epi16(0xF8CB);
  
    const T *in1 = in;
    T *out1 = out;
    out1 += (_size.width << 1) + 2;
    
    __m128i vWeightTable_0 = _mm_loadu_si128((__m128i*)((unsigned short *)WeightTable));
    __m128i vWeightTable_1 = _mm_loadu_si128((__m128i*)((unsigned short *)WeightTable + 8));
    __m128i vWeightTable_2 = _mm_loadu_si128((__m128i*)((unsigned short *)WeightTable + 16));
    __m128i vWeightTable_3 = vWeightTable_1;
    __m128i vWeightTable_4 = vWeightTable_0;
  
    __m128i vZeros = _mm_set1_epi16(0);
    __m128i vWeightTable_0_h = _mm_unpackhi_epi16(vWeightTable_0, vZeros);
    __m128i vWeightTable_0_l = _mm_unpacklo_epi16(vWeightTable_0, vZeros);
    __m128i vWeightTable_1_h = _mm_unpackhi_epi16(vWeightTable_1, vZeros);
    __m128i vWeightTable_1_l = _mm_unpacklo_epi16(vWeightTable_1, vZeros);
    __m128i vWeightTable_2_h = _mm_unpackhi_epi16(vWeightTable_2, vZeros);
    __m128i vWeightTable_2_l = _mm_unpacklo_epi16(vWeightTable_2, vZeros);
    __m128i vWeightTable_3_h = vWeightTable_1_h;
    __m128i vWeightTable_3_l = vWeightTable_1_l;
    __m128i vWeightTable_4_h = vWeightTable_0_h;
    __m128i vWeightTable_4_l = vWeightTable_0_l;
  
    int nHeghtMinus4 = _size.height-4;
    
    for(int j = 0; j < nHeghtMinus4; j++)
    {
      __m128i row1_a = _mm_loadu_si128((__m128i*)((unsigned short *)in1));                      //load first 16 values from row 1
      __m128i row1_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + 8));                 //load second 16 values from row1
      __m128i row2_a = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + _size.width));              //load first 16 values from row 2
      __m128i row2_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + _size.width + 8));          //load second 16 values from row 2
      __m128i row3_a = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 1)));       //load first 16 values from row 3
      __m128i row3_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 1) + 8));
      __m128i row4_a = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width * 3)));       //load first 16 values from row 3
      __m128i row4_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width * 3) + 8));
      __m128i row5_a = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 2)));       //load first 16 values from row 3
      __m128i row5_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 2) + 8));
  
      for(int i = 0; i < _size.width; i+=8)
      {
        __m128i vWeight;
        __m128i vSum;
        __m128i vSum0_0, vSum0_1, vSum0_2, vSum0_3, vSum0_4, vSum0_5, vSum0_6, vSum0_7;
        __m128i vSum1_0, vSum1_1, vSum1_2, vSum1_3, vSum1_4, vSum1_5, vSum1_6, vSum1_7;
        __m128i vSum2_0, vSum2_1, vSum2_2, vSum2_3, vSum2_4, vSum2_5, vSum2_6, vSum2_7;
        __m128i vSum3_0, vSum3_1, vSum3_2, vSum3_3, vSum3_4, vSum3_5, vSum3_6, vSum3_7;
        __m128i vSum4_0, vSum4_1, vSum4_2, vSum4_3, vSum4_4, vSum4_5, vSum4_6, vSum4_7;
        __m128i vec0_0, vec0_1, vec0_2, vec0_3, vec0_4, vec0_5, vec0_6, vec0_7;
        __m128i vec1_0, vec1_1, vec1_2, vec1_3, vec1_4, vec1_5, vec1_6, vec1_7;
        __m128i vec2_0, vec2_1, vec2_2, vec2_3, vec2_4, vec2_5, vec2_6, vec2_7;
        __m128i vec3_0, vec3_1, vec3_2, vec3_3, vec3_4, vec3_5, vec3_6, vec3_7;
        __m128i vec4_0, vec4_1, vec4_2, vec4_3, vec4_4, vec4_5, vec4_6, vec4_7;
        unsigned short Sum[8] = {0};
        __m128i vOut;
  
        vec0_0 = row1_a;
        vec1_0 = row2_a;
        vec2_0 = row3_a;
        vec3_0 = row4_a;
        vec4_0 = row5_a;
  
        vec0_1 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 1));
        vec1_1 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 1));
        vec2_1 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 1));
        vec3_1 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 1));
        vec4_1 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 1));
  
        vec0_2 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 2));
        vec1_2 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 2));
        vec2_2 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 2));
        vec3_2 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 2));
        vec4_2 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 2));
  
        vec0_3 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 3));
        vec1_3 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 3));
        vec2_3 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 3));
        vec3_3 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 3));
        vec4_3 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 3));
  
        vec0_4 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 4));
        vec1_4 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 4));
        vec2_4 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 4));
        vec3_4 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 4));
        vec4_4 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 4));
  
        vec0_5 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 5));
        vec1_5 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 5));
        vec2_5 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 5));
        vec3_5 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 5));
        vec4_5 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 5));
  
        vec0_6 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 6));
        vec1_6 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 6));
        vec2_6 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 6));
        vec3_6 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 6));
        vec4_6 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 6));
  
        vec0_7 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + 7));
        vec1_7 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + _size.width + 7));
        vec2_7 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 1) + 7));
        vec3_7 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width * 3) + 7));
        vec4_7 = _mm_loadu_si128((__m128i*)((unsigned short*)in1 + (_size.width << 2) + 7));
  
        {
  
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_0, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_0, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_0, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_0, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_0, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_0, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_0, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_0, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_0, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_0, vZeros));
  
          vSum0_0 = _mm_add_epi32(temp0, temp0a);
          vSum1_0 = _mm_add_epi32(temp1, temp1a);
          vSum2_0 = _mm_add_epi32(temp2, temp2a);
          vSum3_0 = _mm_add_epi32(temp3, temp3a);
          vSum4_0 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_1, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_1, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_1, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_1, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_1, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_1, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_1, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_1, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_1, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_1, vZeros));
  
          vSum0_1 = _mm_add_epi32(temp0, temp0a);
          vSum1_1 = _mm_add_epi32(temp1, temp1a);
          vSum2_1 = _mm_add_epi32(temp2, temp2a);
          vSum3_1 = _mm_add_epi32(temp3, temp3a);
          vSum4_1 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_2, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_2, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_2, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_2, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_2, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_2, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_2, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_2, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_2, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_2, vZeros));
  
          vSum0_2 = _mm_add_epi32(temp0, temp0a);
          vSum1_2 = _mm_add_epi32(temp1, temp1a);
          vSum2_2 = _mm_add_epi32(temp2, temp2a);
          vSum3_2 = _mm_add_epi32(temp3, temp3a);
          vSum4_2 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_3, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_3, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_3, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_3, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_3, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_3, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_3, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_3, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_3, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_3, vZeros));
  
          vSum0_3 = _mm_add_epi32(temp0, temp0a);
          vSum1_3 = _mm_add_epi32(temp1, temp1a);
          vSum2_3 = _mm_add_epi32(temp2, temp2a);
          vSum3_3 = _mm_add_epi32(temp3, temp3a);
          vSum4_3 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_4, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_4, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_4, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_4, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_4, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_4, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_4, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_4, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_4, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_4, vZeros));
  
          vSum0_4 = _mm_add_epi32(temp0, temp0a);
          vSum1_4 = _mm_add_epi32(temp1, temp1a);
          vSum2_4 = _mm_add_epi32(temp2, temp2a);
          vSum3_4 = _mm_add_epi32(temp3, temp3a);
          vSum4_4 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_5, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_5, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_5, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_5, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_5, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_5, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_5, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_5, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_5, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_5, vZeros));
  
          vSum0_5 = _mm_add_epi32(temp0, temp0a);
          vSum1_5 = _mm_add_epi32(temp1, temp1a);
          vSum2_5 = _mm_add_epi32(temp2, temp2a);
          vSum3_5 = _mm_add_epi32(temp3, temp3a);
          vSum4_5 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_6, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_6, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_6, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_6, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_6, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_6, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_6, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_6, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_6, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_6, vZeros));
  
          vSum0_6 = _mm_add_epi32(temp0, temp0a);
          vSum1_6 = _mm_add_epi32(temp1, temp1a);
          vSum2_6 = _mm_add_epi32(temp2, temp2a);
          vSum3_6 = _mm_add_epi32(temp3, temp3a);
          vSum4_6 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        {
          __m128i temp0, temp1, temp2, temp3, temp4;
          temp0 = muly_32(vWeightTable_0_l, _mm_unpacklo_epi16(vec0_7, vZeros));
          temp1 = muly_32(vWeightTable_1_l, _mm_unpacklo_epi16(vec1_7, vZeros));
          temp2 = muly_32(vWeightTable_2_l, _mm_unpacklo_epi16(vec2_7, vZeros));
          temp3 = muly_32(vWeightTable_3_l, _mm_unpacklo_epi16(vec3_7, vZeros));
          temp4 = muly_32(vWeightTable_4_l, _mm_unpacklo_epi16(vec4_7, vZeros));
  
          __m128i temp0a, temp1a, temp2a, temp3a, temp4a;
          temp0a = muly_32(vWeightTable_0_h, _mm_unpackhi_epi16(vec0_7, vZeros));
          temp1a = muly_32(vWeightTable_1_h, _mm_unpackhi_epi16(vec1_7, vZeros));
          temp2a = muly_32(vWeightTable_2_h, _mm_unpackhi_epi16(vec2_7, vZeros));
          temp3a = muly_32(vWeightTable_3_h, _mm_unpackhi_epi16(vec3_7, vZeros));
          temp4a = muly_32(vWeightTable_4_h, _mm_unpackhi_epi16(vec4_7, vZeros));
  
          vSum0_7 = _mm_add_epi32(temp0, temp0a);
          vSum1_7 = _mm_add_epi32(temp1, temp1a);
          vSum2_7 = _mm_add_epi32(temp2, temp2a);
          vSum3_7 = _mm_add_epi32(temp3, temp3a);
          vSum4_7 = _mm_add_epi32(temp4, temp4a);
  
        }
  
        __m128i vSum0, vSum1, vSum2, vSum3, vSum4, vSum5, vSum6, vSum7;
        vSum0 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_0, vSum1_0), _mm_add_epi32(vSum2_0, vSum3_0)), vSum4_0);
        vSum1 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_1, vSum1_1), _mm_add_epi32(vSum2_1, vSum3_1)), vSum4_1);
        vSum2 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_2, vSum1_2), _mm_add_epi32(vSum2_2, vSum3_2)), vSum4_2);
        vSum3 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_3, vSum1_3), _mm_add_epi32(vSum2_3, vSum3_3)), vSum4_3);
        vSum4 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_4, vSum1_4), _mm_add_epi32(vSum2_4, vSum3_4)), vSum4_4);
        vSum5 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_5, vSum1_5), _mm_add_epi32(vSum2_5, vSum3_5)), vSum4_5);
        vSum6 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_6, vSum1_6), _mm_add_epi32(vSum2_6, vSum3_6)), vSum4_6);
        vSum7 = _mm_add_epi32(_mm_add_epi32(_mm_add_epi32(vSum0_7, vSum1_7), _mm_add_epi32(vSum2_7, vSum3_7)), vSum4_7);
  
        vSum0 = _mm_add_epi32(vSum0, _mm_srli_si128(vSum0, 8));
        vSum0 = _mm_add_epi32(vSum0, _mm_srli_si128(vSum0, 4));
        Sum[0] = (unsigned short)(_mm_cvtsi128_si32(vSum0) >> 16);
  
        vSum1 = _mm_add_epi32(vSum1, _mm_srli_si128(vSum1, 8));
        vSum1 = _mm_add_epi32(vSum1, _mm_srli_si128(vSum1, 4));
        Sum[1] = (unsigned short)(_mm_cvtsi128_si32(vSum1) >> 16);
  
        vSum2 = _mm_add_epi32(vSum2, _mm_srli_si128(vSum2, 8));
        vSum2 = _mm_add_epi32(vSum2, _mm_srli_si128(vSum2, 4));
        Sum[2] = (unsigned short)(_mm_cvtsi128_si32(vSum2) >> 16);
  
        vSum3 = _mm_add_epi32(vSum3, _mm_srli_si128(vSum3, 8));
        vSum3 = _mm_add_epi32(vSum3, _mm_srli_si128(vSum3, 4));
        Sum[3] = (unsigned short)(_mm_cvtsi128_si32(vSum3) >> 16);
  
        vSum4 = _mm_add_epi32(vSum4, _mm_srli_si128(vSum4, 8));
        vSum4 = _mm_add_epi32(vSum4, _mm_srli_si128(vSum4, 4));
        Sum[4] = (unsigned short)(_mm_cvtsi128_si32(vSum4) >> 16);
  
        vSum5 = _mm_add_epi32(vSum5, _mm_srli_si128(vSum5, 8));
        vSum5 = _mm_add_epi32(vSum5, _mm_srli_si128(vSum5, 4));
        Sum[5] = (unsigned short)(_mm_cvtsi128_si32(vSum5) >> 16);
  
        vSum6 = _mm_add_epi32(vSum6, _mm_srli_si128(vSum6, 8));
        vSum6 = _mm_add_epi32(vSum6, _mm_srli_si128(vSum6, 4));
        Sum[6] = (unsigned short)(_mm_cvtsi128_si32(vSum6) >> 16);
  
        vSum7 = _mm_add_epi32(vSum7, _mm_srli_si128(vSum7, 8));
        vSum7 = _mm_add_epi32(vSum7, _mm_srli_si128(vSum7, 4));
        Sum[7] = (unsigned short)(_mm_cvtsi128_si32(vSum7) >> 16);
  
        vSum = _mm_loadu_si128((__m128i*)Sum);
        __m128i temp = muly_32((_mm_unpackhi_epi16(vSum, vZeros)), _mm_unpackhi_epi16(vWeight_sum, vZeros));
        __m128i temp1 = muly_32((_mm_unpacklo_epi16(vSum, vZeros)), _mm_unpacklo_epi16(vWeight_sum, vZeros));
        __m128i tempOut = _mm_srli_epi32(temp, 16);
        __m128i tempOut1 = _mm_srli_epi32(temp1, 16);
        
        vOut = _mm_packs_epi32(tempOut1, tempOut);
  
        _mm_storeu_si128((__m128i*)out1, vOut);
  
        row1_a = row1_b;
        row2_a = row2_b;
        row3_a = row3_b;
        row4_a = row4_b;
        row5_a = row5_b;
        in1 = in1 + 8;
        row1_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + 8));
        row2_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + _size.width + 8));
        row3_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 1) + 8));
        row4_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width * 3) + 8));
        row5_b = _mm_loadu_si128((__m128i*)((unsigned short *)in1 + (_size.width << 2) + 8));
  
        out1 += 8;
  
      }
    }
  }
  
#elif ARM_OPT

  int tempHeight = _size.height-2;
  int tempwidth = _size.width-2;
  
  if(_discreteGaussian.getStandardDeviation() == 0.5)
  {
    /*table contents = round(product * pow(2,16))*/
    uint16_t WeightTable[40] = {0x0000, 0x0002, 0x000E, 0x0002, 0x0000, 0x0000, 0x0000, 0x0000,\
                                0x0002, 0x02FC, 0x160E, 0x02FC, 0x0002, 0x0000, 0x0000, 0x0000,\
                                0x000E, 0x160E, 0xA2FA, 0x160E, 0x000E, 0x0000, 0x0000, 0x0000,\
                                0x0002, 0x02FC, 0x160E, 0x02FC, 0x0002, 0x0000, 0x0000, 0x0000,\
                                0x0000, 0x0002, 0x000E, 0x0002, 0x0000, 0x0000, 0x0000, 0x0000};
  
    uint16x8_t vWeightTable;
    /* vWeight_sum = (1/weight_sum) << 16 */
    uint16x8_t vWeight_sum = vdupq_n_u16(0xF8CB);
  
    const T *in1 = in;
    T *out1 = out;
    out1 += (_size.width << 1) + 2;
    
    uint16x8_t vWeightTable_0 = vld1q_u16((uint16_t *)WeightTable);
    uint16x8_t vWeightTable_1 = vld1q_u16(((uint16_t *)WeightTable) + 8);
    uint16x8_t vWeightTable_2 = vld1q_u16(((uint16_t *)WeightTable) + 16);
    uint16x8_t vWeightTable_3 = vWeightTable_1;
    uint16x8_t vWeightTable_4 = vWeightTable_0;
    
    uint16x4_t vWeightTable_0_h = vget_high_u16(vWeightTable_0);
    uint16x4_t vWeightTable_0_l = vget_low_u16(vWeightTable_0);
    uint16x4_t vWeightTable_1_h = vget_high_u16(vWeightTable_1);
    uint16x4_t vWeightTable_1_l = vget_low_u16(vWeightTable_1);
    uint16x4_t vWeightTable_2_h = vget_high_u16(vWeightTable_2);
    uint16x4_t vWeightTable_2_l = vget_low_u16(vWeightTable_2);
    uint16x4_t vWeightTable_3_h = vWeightTable_1_h;
    uint16x4_t vWeightTable_3_l = vWeightTable_1_l;
    uint16x4_t vWeightTable_4_h = vWeightTable_0_h;
    uint16x4_t vWeightTable_4_l = vWeightTable_0_l;
  
    int nHeghtMinus4 = _size.height-4;
    
    for(int j = 0; j < nHeghtMinus4; j++)
    { 
      uint16x8_t row1_a = vld1q_u16((uint16_t *)in1);                      //load first 16 values from row 1
      uint16x8_t row1_b = vld1q_u16((uint16_t *)in1 + 8);                 //load second 16 values from row1
      uint16x8_t row2_a = vld1q_u16((uint16_t *)in1 + _size.width);              //load first 16 values from row 2
      uint16x8_t row2_b = vld1q_u16((uint16_t *)in1 + _size.width + 8);          //load second 16 values from row 2
      uint16x8_t row3_a = vld1q_u16((uint16_t *)in1 + (_size.width << 1));       //load first 16 values from row 3
      uint16x8_t row3_b = vld1q_u16((uint16_t *)in1 + (_size.width << 1) + 8);    
      uint16x8_t row4_a = vld1q_u16((uint16_t *)in1 + (_size.width * 3));       //load first 16 values from row 3
      uint16x8_t row4_b = vld1q_u16((uint16_t *)in1 + (_size.width * 3) + 8);
      uint16x8_t row5_a = vld1q_u16((uint16_t *)in1 + (_size.width << 2));       //load first 16 values from row 3
      uint16x8_t row5_b = vld1q_u16((uint16_t *)in1 + (_size.width << 2) + 8);
  
      for(int i = 0; i < _size.width; i+=8)
      {
        uint16x8_t vWeight;
        uint16x8_t vSum;
        uint32x4_t vSum0_0, vSum0_1, vSum0_2, vSum0_3, vSum0_4, vSum0_5, vSum0_6, vSum0_7;
        uint32x4_t vSum1_0, vSum1_1, vSum1_2, vSum1_3, vSum1_4, vSum1_5, vSum1_6, vSum1_7;
        uint32x4_t vSum2_0, vSum2_1, vSum2_2, vSum2_3, vSum2_4, vSum2_5, vSum2_6, vSum2_7;
        uint32x4_t vSum3_0, vSum3_1, vSum3_2, vSum3_3, vSum3_4, vSum3_5, vSum3_6, vSum3_7;
        uint32x4_t vSum4_0, vSum4_1, vSum4_2, vSum4_3, vSum4_4, vSum4_5, vSum4_6, vSum4_7;
        uint16x8_t vec0_0, vec0_1, vec0_2, vec0_3, vec0_4, vec0_5, vec0_6, vec0_7;
        uint16x8_t vec1_0, vec1_1, vec1_2, vec1_3, vec1_4, vec1_5, vec1_6, vec1_7;
        uint16x8_t vec2_0, vec2_1, vec2_2, vec2_3, vec2_4, vec2_5, vec2_6, vec2_7;
        uint16x8_t vec3_0, vec3_1, vec3_2, vec3_3, vec3_4, vec3_5, vec3_6, vec3_7;
        uint16x8_t vec4_0, vec4_1, vec4_2, vec4_3, vec4_4, vec4_5, vec4_6, vec4_7;
        uint16_t Sum[8] = {0};
        uint16x8_t vOut;
      
        vec0_0 = row1_a;
        vec1_0 = row2_a;
        vec2_0 = row3_a;
        vec3_0 = row4_a;
        vec4_0 = row5_a;
        
        vec0_1 = vextq_u16(row1_a, row1_b, 1);
        vec1_1 = vextq_u16(row2_a, row2_b, 1);
        vec2_1 = vextq_u16(row3_a, row3_b, 1);
        vec3_1 = vextq_u16(row4_a, row4_b, 1);
        vec4_1 = vextq_u16(row5_a, row5_b, 1);
  
        vec0_2 = vextq_u16(row1_a, row1_b, 2);
        vec1_2 = vextq_u16(row2_a, row2_b, 2);
        vec2_2 = vextq_u16(row3_a, row3_b, 2);
        vec3_2 = vextq_u16(row4_a, row4_b, 2);
        vec4_2 = vextq_u16(row5_a, row5_b, 2);
        
        vec0_3 = vextq_u16(row1_a, row1_b, 3);
        vec1_3 = vextq_u16(row2_a, row2_b, 3);
        vec2_3 = vextq_u16(row3_a, row3_b, 3);
        vec3_3 = vextq_u16(row4_a, row4_b, 3);
        vec4_3 = vextq_u16(row5_a, row5_b, 3);
  
        vec0_4 = vextq_u16(row1_a, row1_b, 4);
        vec1_4 = vextq_u16(row2_a, row2_b, 4);
        vec2_4 = vextq_u16(row3_a, row3_b, 4);
        vec3_4 = vextq_u16(row4_a, row4_b, 4);
        vec4_4 = vextq_u16(row5_a, row5_b, 4);
        
        vec0_5 = vextq_u16(row1_a, row1_b, 5);
        vec1_5 = vextq_u16(row2_a, row2_b, 5);
        vec2_5 = vextq_u16(row3_a, row3_b, 5);
        vec3_5 = vextq_u16(row4_a, row4_b, 5);
        vec4_5 = vextq_u16(row5_a, row5_b, 5);
        
        vec0_6 = vextq_u16(row1_a, row1_b, 6);
        vec1_6 = vextq_u16(row2_a, row2_b, 6);
        vec2_6 = vextq_u16(row3_a, row3_b, 6);
        vec3_6 = vextq_u16(row4_a, row4_b, 6);
        vec4_6 = vextq_u16(row5_a, row5_b, 6);
        
        vec0_7 = vextq_u16(row1_a, row1_b, 7);
        vec1_7 = vextq_u16(row2_a, row2_b, 7);
        vec2_7 = vextq_u16(row3_a, row3_b, 7);
        vec3_7 = vextq_u16(row4_a, row4_b, 7);
        vec4_7 = vextq_u16(row5_a, row5_b, 7);
  
        {  
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_0));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_0));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_0));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_0));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_0));
          
          vSum0_0 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_0)); 
          vSum1_0 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_0));
          vSum2_0 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_0));
          vSum3_0 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_0));
          vSum4_0 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_0));
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_1));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_1));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_1));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_1));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_1));
          
          vSum0_1 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_1)); 
          vSum1_1 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_1));
          vSum2_1 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_1));
          vSum3_1 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_1));
          vSum4_1 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_1));
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_2));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_2));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_2));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_2));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_2));
          
          vSum0_2 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_2)); 
          vSum1_2 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_2));
          vSum2_2 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_2));
          vSum3_2 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_2));
          vSum4_2 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_2));   
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_3));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_3));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_3));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_3));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_3));
          
          vSum0_3 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_3)); 
          vSum1_3 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_3));
          vSum2_3 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_3));
          vSum3_3 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_3));
          vSum4_3 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_3)); 
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_4));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_4));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_4));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_4));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_4));
          
          vSum0_4 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_4)); 
          vSum1_4 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_4));
          vSum2_4 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_4));
          vSum3_4 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_4));
          vSum4_4 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_4)); 
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_5));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_5));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_5));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_5));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_5));
          
          vSum0_5 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_5)); 
          vSum1_5 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_5));
          vSum2_5 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_5));
          vSum3_5 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_5));
          vSum4_5 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_5));
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_6));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_6));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_6));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_6));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_6));
          
          vSum0_6 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_6)); 
          vSum1_6 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_6));
          vSum2_6 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_6));
          vSum3_6 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_6));
          vSum4_6 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_6));
          
        }
        
        {
  
          uint32x4_t temp0, temp1, temp2, temp3, temp4;
          temp0 = vmull_u16(vWeightTable_0_l, vget_low_u16(vec0_7));
          temp1 = vmull_u16(vWeightTable_1_l, vget_low_u16(vec1_7));
          temp2 = vmull_u16(vWeightTable_2_l, vget_low_u16(vec2_7));
          temp3 = vmull_u16(vWeightTable_3_l, vget_low_u16(vec3_7));
          temp4 = vmull_u16(vWeightTable_4_l, vget_low_u16(vec4_7));
          
          vSum0_7 = vmlal_u16(temp0, vWeightTable_0_h, vget_high_u16(vec0_7)); 
          vSum1_7 = vmlal_u16(temp1, vWeightTable_1_h, vget_high_u16(vec1_7));
          vSum2_7 = vmlal_u16(temp2, vWeightTable_2_h, vget_high_u16(vec2_7));
          vSum3_7 = vmlal_u16(temp3, vWeightTable_3_h, vget_high_u16(vec3_7));
          vSum4_7 = vmlal_u16(temp4, vWeightTable_4_h, vget_high_u16(vec4_7));
          
        }
  
        uint32x4_t vSum0, vSum1, vSum2, vSum3, vSum4, vSum5, vSum6, vSum7;
        {
          uint32x4_t t1 = vaddq_u32(vSum0_0, vSum1_0);
          uint32x4_t t2 = vaddq_u32(vSum2_0, vSum3_0);
          vSum0 = vaddq_u32(vaddq_u32(t1, t2), vSum4_0);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_1, vSum1_1);
          uint32x4_t t2 = vaddq_u32(vSum2_1, vSum3_1);
          vSum1 = vaddq_u32(vaddq_u32(t1, t2), vSum4_1);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_2, vSum1_2);
          uint32x4_t t2 = vaddq_u32(vSum2_2, vSum3_2);
          vSum2 = vaddq_u32(vaddq_u32(t1, t2), vSum4_2);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_3, vSum1_3);
          uint32x4_t t2 = vaddq_u32(vSum2_3, vSum3_3);
          vSum3 = vaddq_u32(vaddq_u32(t1, t2), vSum4_3);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_4, vSum1_4);
          uint32x4_t t2 = vaddq_u32(vSum2_4, vSum3_4);
          vSum4 = vaddq_u32(vaddq_u32(t1, t2), vSum4_4);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_5, vSum1_5);
          uint32x4_t t2 = vaddq_u32(vSum2_5, vSum3_5);
          vSum5 = vaddq_u32(vaddq_u32(t1, t2), vSum4_5);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_6, vSum1_6);
          uint32x4_t t2 = vaddq_u32(vSum2_6, vSum3_6);
          vSum6 = vaddq_u32(vaddq_u32(t1, t2), vSum4_6);
        }
        {
          uint32x4_t t1 = vaddq_u32(vSum0_7, vSum1_7);
          uint32x4_t t2 = vaddq_u32(vSum2_7, vSum3_7);
          vSum7 = vaddq_u32(vaddq_u32(t1, t2), vSum4_7);
        }
        
        Sum[0] = (vgetq_lane_u32(vSum0, 0) + vgetq_lane_u32(vSum0, 1) + vgetq_lane_u32(vSum0, 2) + vgetq_lane_u32(vSum0, 3)) >> 16;
        Sum[1] = (vgetq_lane_u32(vSum1, 0) + vgetq_lane_u32(vSum1, 1) + vgetq_lane_u32(vSum1, 2) + vgetq_lane_u32(vSum1, 3)) >> 16;
        Sum[2] = (vgetq_lane_u32(vSum2, 0) + vgetq_lane_u32(vSum2, 1) + vgetq_lane_u32(vSum2, 2) + vgetq_lane_u32(vSum2, 3)) >> 16;
        Sum[3] = (vgetq_lane_u32(vSum3, 0) + vgetq_lane_u32(vSum3, 1) + vgetq_lane_u32(vSum3, 2) + vgetq_lane_u32(vSum3, 3)) >> 16;
        Sum[4] = (vgetq_lane_u32(vSum4, 0) + vgetq_lane_u32(vSum4, 1) + vgetq_lane_u32(vSum4, 2) + vgetq_lane_u32(vSum4, 3)) >> 16;
        Sum[5] = (vgetq_lane_u32(vSum5, 0) + vgetq_lane_u32(vSum5, 1) + vgetq_lane_u32(vSum5, 2) + vgetq_lane_u32(vSum5, 3)) >> 16;
        Sum[6] = (vgetq_lane_u32(vSum6, 0) + vgetq_lane_u32(vSum6, 1) + vgetq_lane_u32(vSum6, 2) + vgetq_lane_u32(vSum6, 3)) >> 16;
        Sum[7] = (vgetq_lane_u32(vSum7, 0) + vgetq_lane_u32(vSum7, 1) + vgetq_lane_u32(vSum7, 2) + vgetq_lane_u32(vSum7, 3)) >> 16;
  
        vSum = vld1q_u16(Sum);
        uint32x4_t temp = vmull_u16(vget_high_u16(vSum), vget_high_u16(vWeight_sum));
        uint32x4_t temp1 = vmull_u16(vget_low_u16(vSum), vget_low_u16(vWeight_sum));
        uint32x4_t tempOut = vshrq_n_u32(temp, 16);
        uint32x4_t tempOut1 = vshrq_n_u32(temp1, 16);
        vOut = vcombine_u16(vqmovn_u32(tempOut1), vqmovn_u32(tempOut));
  
        vst1q_u16((uint16_t *)out1, vOut);
  
        row1_a = row1_b;
        row2_a = row2_b;
        row3_a = row3_b;
        row4_a = row4_b;
        row5_a = row5_b;
        in1 = in1 + 8;
        row1_b = vld1q_u16((uint16_t *)in1 + 8);
        row2_b = vld1q_u16((uint16_t *)in1 + _size.width + 8);
        row3_b = vld1q_u16((uint16_t *)in1 + (_size.width << 1) + 8);
        row4_b = vld1q_u16((uint16_t *)in1 + (_size.width * 3) + 8);
        row5_b = vld1q_u16((uint16_t *)in1 + (_size.width << 2) + 8);
        
        out1 += 8;
  
      }
    }
  }
#else
  for (int j = 0; j < _size.height; j++) {
    for (int i = 0; i < _size.width; i++) {
      int p = j*_size.width + i;
      float weight_sum = 0;
      float sum = 0;
      for (int k = -2; k <= 2; k++) {
        for (int m = -2; m <= 2; m++) {
          int i2 = i + m;
          int j2 = j + k;
          
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) {
            int q = j2*_size.width + i2;
            float weight = _discreteGaussian.valueAt(k)*_discreteGaussian.valueAt(m);
            weight_sum += weight;
            sum += weight * in[q];
          }
        }
      }
      out[p] = ((T)(sum / weight_sum));
    }
  }
#endif
  return true;
}

bool SmoothFilter::_filter(const FramePtr &in, FramePtr &out)
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