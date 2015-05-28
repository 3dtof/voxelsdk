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
  
/**
 * \addtogroup Flt
 * @{
 */

template <typename FrameType>
class FilterSet;

template <typename FrameType>
class FilterSetIterator
{
public:
  int i;
  const FilterSet<FrameType> &s;
  int index;
  
  FilterSetIterator(const FilterSet<FrameType> &s, int i = 0);
  inline FilterSetIterator &operator++(int);
  inline FilterSetIterator &operator++();
  inline const FilterPtr operator *();
  inline bool operator !=(const FilterSetIterator &other) const;
  inline bool operator ==(const FilterSetIterator &other) const;
};

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
  int addFilter(FilterPtr p, int beforeFilterIndex = -1);
  FilterPtr getFilter(int index) const;
  bool removeFilter(int index);
  bool removeAllFilters();
  
  inline SizeType size() const { return _indices.size(); }
  
  // Populate one entry in 'seq' before calling this function. That entry will be used
  // as input to the first filter and then onwards till the last filter. Each filter will
  // append one entry which contains the filter frame
  bool applyFilter(FrameSequence &seq);
  
  void reset();
  
  FilterSetIterator<FrameType> begin() const
  {
    return FilterSetIterator<FrameType>(*this, 0); 
  }
  
  FilterSetIterator<FrameType> end() const
  {
    return FilterSetIterator<FrameType>(*this, _indices.size());
  }
  
  friend class FilterSetIterator<FrameType>;
};

template <typename FrameType>
int FilterSet<FrameType>::addFilter(FilterPtr p, int beforeFilterIndex)
{
  Lock<Mutex> _(_accessMutex);
  
  if( beforeFilterIndex == -1)
    _indices.push_back(_filterCounter);
  else
  {
    auto found = -1;
    for(auto i = 0; i < _indices.size(); i++)
      if(_indices[i] == beforeFilterIndex)
        found = i;
      
    if(found < 0)
      return -1; // -1 => invalid filter index
    
    _indices.insert(_indices.begin() + found, _filterCounter);
  }
  
  _filters[_filterCounter] = p;
  
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

template <typename FrameType>
void FilterSet<FrameType>::reset()
{
  for(auto f: *this)
    f->reset();
}

template <typename FrameType>
FilterSetIterator<FrameType>::FilterSetIterator(const FilterSet<FrameType> &s, int i): s(s), i(i) 
{
  if(i < s._indices.size())
    index = s._indices[i];
}

template <typename FrameType>
inline FilterSetIterator<FrameType> &FilterSetIterator<FrameType>::operator++(int) 
{
  i++;
  
  if(i < s._indices.size())
    index = s._indices[i];
  
  return *this;
}

template <typename FrameType>
inline FilterSetIterator<FrameType> &FilterSetIterator<FrameType>::operator++() 
{
  i++;
  
  if(i < s._indices.size())
    index = s._indices[i];
  
  return *this;
}

template <typename FrameType>
inline const FilterPtr FilterSetIterator<FrameType>::operator *()
{
  if(i < s._indices.size())
  {
    return s._filters.at(index);
  }
  else
    return nullptr;
}

template <typename FrameType>
inline bool FilterSetIterator<FrameType>::operator !=(const FilterSetIterator &other) const
{
  return !operator==(other);
}

template <typename FrameType>
inline bool FilterSetIterator<FrameType>::operator ==(const FilterSetIterator &other) const
{
  return (&s == &other.s) && (i == other.i);
}


/**
 * @}
 */

  
}

#endif //VOXEL_FRAME_FILTER_SET_H