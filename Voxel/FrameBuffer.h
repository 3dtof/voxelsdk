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

template <typename BufferType>
class FrameBufferManager;

// Auto-releases the obtained buffer when FrameBuffer<> instance is terminated
template <typename BufferType>
class FrameBuffer: public Ptr<BufferType>
{
public:
  typedef Ptr<BufferType> BufferPtr;
  
  BufferPtr &_buffer;
  
  FrameBufferManager<BufferType> &_manager;
  
  FrameBuffer(BufferPtr &buffer, FrameBufferManager<BufferType> &manager): _buffer(buffer), _manager(manager), 
    Ptr<BufferType>(nullptr, [this](BufferType *) { this->release(); }) // The deleter gets called when FrameBuffer<> goes out of scope. Deleter releases the held buffer
    {}
  
  inline BufferPtr &operator *() { return _buffer; }
  
  inline void release();
  
  virtual ~FrameBuffer() {}
};
  
// This maintains a minimum number of buffers for use and re-use.
// Call to release() on unused buffers is important for it be either freed or reused 
template <typename BufferType>
class FrameBufferManager
{
public:
  typedef Ptr<BufferType> BufferPtr;
  typedef FrameBuffer<BufferType> FrameBufferType;
  
protected:
  List<BufferPtr> _inUse;
  List<BufferPtr> _available;
  
  SizeType _minimumBufferCount;
  
public:
  FrameBufferManager(SizeType minBufferCount): _minimumBufferCount(minBufferCount)
  {
    for(auto i = 0; i < minBufferCount; i++)
      _available.push_back(BufferPtr());
  }
  
  FrameBufferType get()
  {
    if(_available.size() > 0)
      _inUse.splice(_inUse.begin(), _available, _available.begin());
    else
      _inUse.push_front(BufferPtr());
    
    return FrameBufferType(*_inUse.begin(), *this);
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
  
  virtual ~FrameBufferManager() { clear(); }
};

template <typename BufferType>
inline void FrameBuffer<BufferType>::release()
{
  _manager.release(_buffer);
}


}

#endif // FRAMEBUFFER_H
