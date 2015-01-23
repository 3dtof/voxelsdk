/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FRAME_FILTER_SET_H
#define VOXEL_FRAME_FILTER_SET_H

#include <Filter/Filter.h>

namespace Voxel
{

template <typename FrameType>
class FilterSet
{
protected:
  Vector<int> _indices;
  int _filterCounter;
  Map<int, FilterPtr> _filters;
  
  mutable Mutex _accessMutex;
  
  FrameBufferManager<FrameType> &_frameBufferManager;
  
public:  
  class FrameSequence: public List<FrameBuffer<FrameType>>
  {
  public:
    ~FrameSequence()
    {
      while(this->size())
        this->pop_front();
    }
  };
  
  FilterSet(FrameBufferManager<FrameType> &m): _frameBufferManager(m), _filterCounter(0) { _indices.reserve(10); }
  
  // position = -1 => at the end, otherwise at zero-indexed 'position'
  int addFilter(FilterPtr p, int position = -1);
  FilterPtr getFilter(int index) const;
  bool removeFilter(int index);
  bool removeAllFilters();
  
  inline SizeType size() const { return _indices.size(); }
  
  // Populate one entry in 'seq' before calling this function. That entry will be used
  // as input to the first filter and then onwards till the last filter. Each filter will
  // append one entry which contains the filter frame
  bool applyFilter(FrameSequence &seq);
  
  class Iterator
  {
  public:
    int i;
    const FilterSet &s;
    int index;
    
    Iterator(const FilterSet &s, int i = 0): s(s), i(i) 
    {
      if(i < s._indices.size())
        index = s._indices[i];
    }
    
    inline Iterator &operator++(int) 
    {
      i++;
      
      if(i < s._indices.size())
        index = s._indices[i];
      
      return *this;
    }
    
    inline Iterator &operator++() 
    {
      i++;
      
      if(i < s._indices.size())
        index = s._indices[i];
      
      return *this;
    }
    
    inline const FilterPtr operator *()
    {
      if(i < s._indices.size())
      {
        return s._filters.at(index);
      }
      else
        return nullptr;
    }
    
    inline bool operator !=(const Iterator &other) const
    {
      return !operator==(other);
    }
    
    inline bool operator ==(const Iterator &other) const
    {
      return (&s == &other.s) && (i == other.i);
    }
  };
  
  Iterator begin() const
  {
    return Iterator(*this, 0); 
  }
  
  Iterator end() const
  {
    return Iterator(*this, _indices.size());
  }
};

template <typename FrameType>
int FilterSet<FrameType>::addFilter(FilterPtr p, int position)
{
  Lock<Mutex> _(_accessMutex);
  
  _filters[_filterCounter] = p;
  
  if(position == -1 || position >= _indices.size())
    _indices.push_back(_filterCounter);
  else
    _indices.insert(_indices.begin() + position, _filterCounter);
  
  _frameBufferManager.setMinimumBufferCount(_frameBufferManager.getMinimumBufferCount() + 1);
  
  int i = _filterCounter;
  
  _filterCounter++;
  
  return i;
}

template <typename FrameType>
FilterPtr FilterSet<FrameType>::getFilter(int index) const
{
  Lock<Mutex> _(_accessMutex);
  
  auto x = _filters.find(index);
  
  if(x != _filters.end())
    return x->second;
  else
    return nullptr;
}


template <typename FrameType>
bool FilterSet<FrameType>::removeFilter(int index)
{
  Lock<Mutex> _(_accessMutex);
  
  auto x = _filters.find(index);
  
  if(x != _filters.end())
  {
    for(auto i = 0; i < _indices.size(); i++)
    {
      if(_indices[i] == index)
      {
        _indices.erase(_indices.begin() + i);
        break;
      }
    }
    _filters.erase(x);
    _frameBufferManager.setMinimumBufferCount(_frameBufferManager.getMinimumBufferCount() - 1);
    
    return true;
  }
  
  return false;
}

template <typename FrameType>
bool FilterSet<FrameType>::removeAllFilters()
{
  Lock<Mutex> _(_accessMutex);
  
  _frameBufferManager.setMinimumBufferCount(_frameBufferManager.getMinimumBufferCount() - _filters.size());
  _filters.clear();
  _indices.clear();
  
  return true;
}

template <typename FrameType>
bool FilterSet<FrameType>::applyFilter(FrameSequence &seq)
{
  Lock<Mutex> _(_accessMutex);
  
  if(seq.size() < 1)
  {
    logger(LOG_ERROR) << "Require atleast one frame in the sequence, to apply filters" << std::endl;
    return false;
  }
  
  for(auto f: *this)
  {
    auto g = _frameBufferManager.get();
    
    FramePtr p = std::dynamic_pointer_cast<Frame>(*g);
    
    if(!f->filter(std::dynamic_pointer_cast<Frame>(**seq.begin()), p))
    {
      logger(LOG_ERROR) << "FilterSet: Could not apply filter '" << f->name() << "'" << std::endl;
      return false;
    }
    
    *g = std::dynamic_pointer_cast<FrameType>(p);
    
    if(!*g)
    {
      logger(LOG_ERROR) << "FilterSet: Got an invalid frame from filter '" << f->name() << "'" << std::endl;
      return false;
    }
    
    seq.push_front(g);
  }
  return true;
}


  
}

#endif //VOXEL_FRAME_FILTER_SET_H