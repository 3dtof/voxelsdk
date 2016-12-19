

#include "TemporalMedianFilter.h"

#include <utility>

namespace Voxel
{
  
TemporalMedianFilter::TemporalMedianFilter(uint order, float deadband): Filter("TemporalMedianFilter"), _order(order), _deadband(deadband) 
{
  _addParameters({
    FilterParameterPtr(new UnsignedFilterParameter("order", "Order", "Order of the filter", _order, "", 1, 100)),
    FilterParameterPtr(new FloatFilterParameter("deadband", "Dead band", "Dead band", _deadband, "", 0, 1)),
  });
#ifdef ARM_OPT
  _history = (ByteType **)malloc(_order * sizeof(ByteType *));
  _history[0] = NULL;
  _history[1] = NULL;
  _history[2] = NULL;
  _current = NULL;
#endif
}

void TemporalMedianFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "order")
  {
    if(!_get(f->name(), _order))
    {
      logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'order' parameter" << std::endl;
    }
  }
  else if(f->name() == "deadband")
  {
    if(!_get(f->name(), _deadband))
    {
      logger(LOG_WARNING) << "TemporalMedianFilter: Could not get the recently updated 'deadband' parameter" << std::endl;
    }
  }
}

void TemporalMedianFilter::reset()
{
#ifndef ARM_OPT
  _history.clear(); 
  _current.clear();
#else
  if(_history[0] != NULL)
    free(_history[0]);
  if(_history[1] != NULL)
    free(_history[1]);
  if(_history[2] != NULL)
    free(_history[2]);
  if(_current != NULL)
    free(_current);
  _history[0] = (ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _history[1] = (ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _history[2] = (ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  _current = (ByteType *)malloc(nFrameWidth*nFrameHeight*4);
  memset(_current, 0, nFrameWidth*nFrameHeight*4);
#endif
}

template <typename T>
bool TemporalMedianFilter::_filter(const T *in, T *out)
{
  uint s = _size.width*_size.height;
  
  T *cur, *hist;
#ifdef x86_OPT
  unsigned int transSize = s*sizeof(T);
  if(_current.size() != transSize)
  {
    _current.resize(transSize);
    memset(_current.data(), 0, transSize);
  }
  
  cur = (T *)_current.data();
  
  if(_history.size() < _order)
  {
    Vector<ByteType> h;
    h.resize(transSize);
    
    memcpy(h.data(), in, transSize);
    
    _history.push_back(std::move(h));
    
    memcpy(cur, in, transSize);
    memcpy(out, in, transSize);
  }
  else
  {
    _history.pop_front();
    
    Vector<ByteType> h;
    h.resize(transSize);
    
    memcpy(h.data(), in, transSize);
    
    _history.push_back(std::move(h));
    
    for(auto i = 0; i < s; i++)
    {
      T v;
      _getMedian(i, v);
      if(v > 0 && fabs((float)v - cur[i]) > _deadband * cur[i])
        out[i] = cur[i] = v;
      else
        out[i] = cur[i];
    }
  }

#elif ARM_OPT
  auto number_bytes = s*sizeof(T);
  cur = (T *)_current;

  if(frame_cnt < _order)
  {

    memcpy(_history[frame_cnt], in, number_bytes);
    memcpy(cur, in, number_bytes);
    memcpy(out, in, number_bytes);
    frame_cnt++;
  }
  else
  {

    memcpy(_history[number_frames], in, number_bytes);
    number_frames=(number_frames==2)?0:++number_frames;
    int bits = 0;
    int cnt = 0;
    s = s / 8;
    uint16x8_t frame1;
    uint16x8_t frame2;
    uint16x8_t frame3;
    uint16x8_t zeros ;
    uint16x8_t temp;
    /*--- 0.05 * 2^18 . INORDER TO PERFORM INTEGER MULTIPLICATION(APPROX.) ---*/
    uint16x8_t vdead = vdupq_n_u16(13108); 
    uint16x8_t temp_mul;
    uint16x8_t temp_str;
    uint16x8_t temp_str1;
    uint16x8_t temp_cmp;
    uint16x8_t subq;
    
    for (auto i = 0; i < s; i++)
    {
      zeros = vdupq_n_u16(0);
      /*---  LOADING THE FRAMES ---*/  
      frame1 = vld1q_u16(((uint16_t *)(_history[0])) + bits);
      frame2 = vld1q_u16(((uint16_t *)(_history[1])) + bits);
      frame3 = vld1q_u16(((uint16_t *)(_history[2])) + bits);
      uint16x8_t vcur = vld1q_u16((uint16_t *)cur + bits);
      
      /*--- FINDING THE MEDIAN ---*/
      vmax_min(frame1, frame2);                       
      vmax_min(frame1, frame3);
      vmax_min(frame2, frame3);
      
      /*--- CHECKING WHETHER THE MEDIAN IS ZERO ---*/
      zeros = vcleq_u16(zeros,frame2); 
      
      /*--- FINDING DIFFERENCE BETWEEN CURRENT PIXEL AND THE REFERENCE PIXEL ---*/ 
      subq = vsubq_u16(frame2,vcur);                  
      temp = vreinterpretq_u16_s16(vabsq_s16(vreinterpretq_s16_u16(subq)));  //FINDING THE ABSOLUTE VALUE
      
      /*--- VQDMULHQ WILL MULTIPLY TWO VECTORS(VECTOR COMPRISES OF EIGHT ELEMENT AND EACH ELEMENT SIZE IS 16-BIT) AND STORES THE RESULT IN 
            32-BIT VECTOR AND SHIFTS THE RESULT 16 TIMES TO RIGHT ---*/ 
      temp_mul = (vreinterpretq_u16_s16(vqdmulhq_s16(vreinterpretq_s16_u16(vdead),vreinterpretq_s16_u16(vcur)))); //MULTIPLYING REFERENCE PIXEL AND DEADBAND VALUE  
      temp_mul = vshrq_n_u16(temp_mul,3);   
      
      /*--- CHECKING   fabs((float)v - cur[i]) > _deadband * cur[i]) ---*/
      temp_cmp = vcgtq_u16(temp,temp_mul);
      
      /*--- STORING PROPER value.  fabs((float)v - cur[i]) > _deadband * cur[i])==1 STORE THE MEDIAN VALUE
            ELSE STORE THE REFERENCE PIXEL VALUE ---*/
    
      /*--- TEMP_CMP CONATINS ONES WHEN   v > 0 &&  fabs((float)v - cur[i]) > _deadband * cur[i])==1 ---*/
      temp_cmp = vandq_u16(zeros, temp_cmp);      

      /*--- PARSING ONLY THOSE ELEMENTS WHICH SATISFY THE ABOVE IF STATEMENT ---*/
      temp_str = vandq_u16(temp_cmp, frame2);           

      /*--- PARSING ONLY THOSE ELEMENTS WHICH SATISFY THE ELSE CASE ---*/
      temp_str1 = vandq_u16(vmvnq_u16(temp_cmp), vcur);   
      
      /*--- COMBINING THE RESULTS OF IF AND ELSE STATEMENT ---*/
      temp_str = vorrq_u16(temp_str, temp_str1);           
      
      /*--- STORING THE RESULT IN OUT AND CUR ---*/
      vst1q_u16((uint16_t *)out + bits , temp_str);
      vst1q_u16((uint16_t *)cur + bits , temp_str);
      
      /*--- INCREAMTING THE POINTER TO NEXT 8 PIXELS ---*/
      bits = bits + 8;
    }

  }
#else
  if(_current.size() != s*sizeof(T))
  {
    _current.resize(s*sizeof(T));
    memset(_current.data(), 0, s*sizeof(T));
  }
  
  cur = (T *)_current.data();
  
  if(_history.size() < _order)
  {
    Vector<ByteType> h;
    h.resize(s*sizeof(T));
    
    memcpy(h.data(), in, s*sizeof(T));
    
    _history.push_back(std::move(h));
    
    memcpy(cur, in, s*sizeof(T));
    memcpy(out, in, s*sizeof(T));
  }
  else
  {
    _history.pop_front();
    
    Vector<ByteType> h;
    h.resize(s*sizeof(T));
    
    memcpy(h.data(), in, s*sizeof(T));
    
    _history.push_back(std::move(h));
    
    for(auto i = 0; i < s; i++)
    {
      T v;
      
      _getMedian(i, v);
      
      if(v > 0 && fabs(((float)v - cur[i])/cur[i]) > _deadband)
        out[i] = cur[i] = v;
      else
        out[i] = cur[i];
    }
  }
#endif
  return true;
}

#ifndef ARM_OPT
template <typename T>
void TemporalMedianFilter::_getMedian(IndexType offset, T &value)
{
#ifdef x86_OPT
  if(_order == 3)
  {
    T v[3] = {0}, tempi;
    int temp, isize;
    int k, i = 0;
    int nOffset = offset*sizeof(T);
    for(auto &h: _history)
    {
      T *h1 = (T *)(h.data());

      if(h.size() > nOffset)
      {
        v[i] = (h1[offset]);
        i++;
      }
    }

    int isizeb2 = i/2;
    for(int j = 0; j <= isizeb2; j++)
    {
      for(k = j+1, temp = j; k < i; k++)
      {
        if(v[k] < v[temp])
          temp = k;
      }
      if(temp != j)
      {
        tempi = v[j];
        v[j] = v[temp];
        v[temp] = tempi;
      }
    }
    value = v[i/2];
  }
  else
  {
    Vector<T> v;

    v.reserve(_order);

    for(auto &h: _history)
    {
      T *h1 = (T *)(h.data());
      if(h.size() > offset*sizeof(T))
        v.push_back(h1[offset]);
    }

    std::nth_element(v.begin(), v.begin() + v.size()/2,  v.end());

    value = v[v.size()/2];
  }
#else
  Vector<T> v;
  
  v.reserve(_order);
  
  for(auto &h: _history)
  {
    T *h1 = (T *)(h.data());
    if(h.size() > offset*sizeof(T))
      v.push_back(h1[offset]);
  }
  
  std::nth_element(v.begin(), v.begin() + v.size()/2,  v.end());
  
  value = v[v.size()/2];
#endif
}
#endif

bool TemporalMedianFilter::_filter(const FramePtr &in, FramePtr &out)
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