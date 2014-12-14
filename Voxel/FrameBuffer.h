/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FRAMEBUFFER_H
#define VOXEL_FRAMEBUFFER_H

#include <Ptr.h>
#include <Common.h>

#include <algorithm>

namespace Voxel
{

// This maintains a minimum number of buffers for use and re-use.
// Call to release() on unused buffers is important for it be either freed or reused 
template <typename BufferType>
class FrameBuffer
{
public:
  typedef Ptr<BufferType> BufferPtr;
  
protected:
  List<BufferPtr> _inUse;
  List<BufferPtr> _available;
  
  SizeType _minimumBufferCount;
  
public:
  FrameBuffer(SizeType minBufferCount): _minimumBufferCount(minBufferCount)
  {
    for(auto i = 0; i < minBufferCount; i++)
      _available.push_back(BufferPtr());
  }
  
  BufferPtr &get()
  {
    if(_available.size() > 0)
    {
      _inUse.splice(_inUse.begin(), _available, _available.begin());
      return *_inUse.begin();
    }
    else
    {
      _inUse.push_front(BufferPtr());
      return *_inUse.begin();
    }
  }
  
  bool release(BufferPtr &p)
  {
    auto f = std::find(_inUse.begin(), _inUse.end(), p);
    
    if(f == _inUse.end())
      return false;
    
    if(_available.size() < _minimumBufferCount)
      _available.splice(_available.begin(), _inUse, f);
    else
      _inUse.erase(f);
    
    return true;
  }
  
  void clear()
  {
    _inUse.clear();
    _available.clear();
  }
  
  virtual ~FrameBuffer() { clear(); }
};

}

#endif // FRAMEBUFFER_H
