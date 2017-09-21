
#include "FlypixFilter.h"

#include <memory.h>

namespace Voxel
{

FlypixFilter::FlypixFilter(float thr): Filter("FlypixFilter"), _thr(thr)
{
  _addParameters({
    FilterParameterPtr(new FloatFilterParameter("threshold", "threshold", "gradient threshold", thr, "", 0, 10000))
  });
}

void FlypixFilter::_onSet(const FilterParameterPtr &f)
{
  if(f->name() == "threshold")
  {
    float s;
    if(!_get(f->name(), s))
    {
      logger(LOG_WARNING) << "FlypixFilter: Could not get the recently updated 'threshold' parameter" << std::endl;
    }
    _thr = s;
  }
}

void FlypixFilter::reset() {}

template <typename T>
bool FlypixFilter::_filter(const T *in, T *out)
{
    static float Gx[3][3] = {{ -3,   0,  3},
                             {-10,   0, 10},
                             { -3,   0,  3}};
    static float Gy[3][3] = {{ -3, -10, -3},
                             {  0,   0,  0},
                             {  3,  10,  3}};

  uint s = _size.width*_size.height;
  
  for (int j = 0; j < _size.height; j++) {
    for (int i = 0; i < _size.width; i++) {
      int p = j*_size.width + i;
      float Gx_sum = 0;
      float Gy_sum = 0;
      for (int k = -1; k <= 1; k++) {
        for (int m = -1; m <= 1; m++) {                         
          int i2 = i + m; 
          int j2 = j + k;
          
          if ((j2 >= 0 && j2 < _size.height) && (i2 >= 0 && i2 < _size.width)) {
            int q = j2*_size.width + i2;
            Gx_sum += Gx[k+1][m+1] * in[q];
            Gy_sum += Gy[k+1][m+1] * in[q];
          }
        }
      }
      float sum = sqrt(Gx_sum*Gx_sum + Gy_sum*Gy_sum);
      out[p] = (sum < _thr) ? in[p] : 0;
       
    }
  }

  return true;  
}

bool FlypixFilter::_filter(const FramePtr &in, FramePtr &out)
{
  ToFRawFrame *tofFrame = dynamic_cast<ToFRawFrame *>(in.get());
  DepthFrame *depthFrame = dynamic_cast<DepthFrame *>(in.get());
  
  if((!tofFrame && !depthFrame) || !_prepareOutput(in, out))
  {
    logger(LOG_ERROR) << "FlypixFilter: Input frame type is not ToFRawFrame or DepthFrame or failed get the output ready" << std::endl;
    return false;
  }
  
  if(tofFrame)
  {
    _size = tofFrame->size;
    ToFRawFrame *o = dynamic_cast<ToFRawFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "FlypixFilter: Invalid frame type. Expecting ToFRawFrame." << std::endl;
      return false;
    }
    
   
    uint s = _size.width*_size.height;
    memcpy(o->ambient(), tofFrame->ambient(), s*tofFrame->ambientWordWidth());
    memcpy(o->amplitude(), tofFrame->amplitude(), s*tofFrame->amplitudeWordWidth());
    memcpy(o->flags(), tofFrame->flags(), s*tofFrame->flagsWordWidth());
    
    if(tofFrame->phaseWordWidth() == 2)
      return _filter<uint16_t>((uint16_t *)tofFrame->phase(), (uint16_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 1)
      return _filter<uint8_t>((uint8_t *)tofFrame->phase(), (uint8_t *)o->phase());
    else if(tofFrame->phaseWordWidth() == 4)
      return _filter<uint32_t>((uint32_t *)tofFrame->phase(), (uint32_t *)o->phase());
    else
      return false;
  }
  else if(depthFrame)
  {
    _size = depthFrame->size;
    DepthFrame *o = dynamic_cast<DepthFrame *>(out.get());
    
    if(!o)
    {
      logger(LOG_ERROR) << "FlypixFilter: Invalid frame type. Expecting DepthFrame." << std::endl;
      return false;
    }
    
    o->amplitude = depthFrame->amplitude;
    
    return _filter<float>(depthFrame->depth.data(), o->depth.data());
  }
  else
    return false;
}
  
}
