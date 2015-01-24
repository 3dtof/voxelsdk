/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FILTER_PARAMETER_H
#define VOXEL_FILTER_PARAMETER_H

#include "Common.h"

namespace Voxel
{
  
/**
 * \addtogroup Flt
 * @{
 */
 
class FilterParameter
{
protected:
  String _name, _displayName, _description;
  
public:
  FilterParameter(const String &name, const String &displayName, const String &description): 
    _name(name), _displayName(displayName), _description(description) {}
    
  inline const String &name() const { return _name; }
  inline const String &displayName() const { return _displayName; }
  inline const String &description() const { return _description; }
  
  virtual ~FilterParameter() {}
};

typedef Ptr<FilterParameter> FilterParameterPtr;

template <typename T>
class FilterParameterTemplate: public FilterParameter
{
protected:
  T _value;
  
public:
  FilterParameterTemplate(const String &name, const String &displayName, const String &description, const T &defaultValue):
    FilterParameter(name, displayName, description), _value(defaultValue) {}
  virtual bool get(T &v) const { v = _value; return true; }
  virtual bool set(const T &v) 
  { 
    if(validate(v))
    {
      _value = v; 
      return true;
    }
    else
      return false;
  }
  
  virtual bool validate(const T &v) const = 0;
  
  virtual ~FilterParameterTemplate() {}
};

template <typename T>
class FilterParameterRangeTemplate: public FilterParameterTemplate<T>
{
protected:
  T _lowerLimit, _upperLimit;
  String _unit;
public:
  FilterParameterRangeTemplate(const String &name, const String &displayName, const String &description, const T &defaultValue,
    const String &unit,
    const T &lowerLimit, const T &upperLimit): 
    FilterParameterTemplate<T>(name, displayName, description, defaultValue), 
    _unit(unit),
    _lowerLimit(lowerLimit), _upperLimit(upperLimit) {}
    
  inline const T &lowerLimit() const { return _lowerLimit; }
  inline const T &upperLimit() const { return _upperLimit; }
  
  inline const String &unit() const { return _unit; }
  
  virtual bool validate(const T &v) const
  {
    return (v >= _lowerLimit && v <= _upperLimit);
  }
  
  virtual ~FilterParameterRangeTemplate() {}
};

typedef FilterParameterRangeTemplate<uint> UnsignedFilterParameter;
typedef FilterParameterRangeTemplate<int> SignedFilterParameter;
typedef FilterParameterRangeTemplate<float> FloatFilterParameter;


template <typename T>
class FilterParameterEnumTemplate: public FilterParameterTemplate<T>
{
protected:
  Vector<T> _values;
  Vector<String> _valueMeaning, _valueDescription;
  
public:
  FilterParameterEnumTemplate(const String &name, const String &displayName, const String &description, const T &defaultValue,
    const Vector<T> &values, const Vector<String> &valueMeaning, const Vector<String> &valueDescription):
    FilterParameterTemplate<T>(name, displayName, description, defaultValue),
    _values(values), _valueMeaning(valueMeaning), _valueDescription(valueDescription) {}
    
  inline const Vector<String> &valueMeaning() const { return _valueMeaning; }
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
  inline const Vector<T> &values() const { return _values; }
  
  virtual bool validate(const T &v) const
  {
    for(auto t: _values)
    {
      if(t == v)
        return true;
    }
    return false;
  }
  
  virtual ~FilterParameterEnumTemplate() {}
};

typedef FilterParameterEnumTemplate<int> EnumFilterParameter;

class BoolFilterParameter: public FilterParameterEnumTemplate<bool>
{
public:
  BoolFilterParameter(const String &name, const String &displayName, const String &description, const bool &defaultValue,
    const Vector<String> &valueMeaning, const Vector<String> &valueDescription):
  FilterParameterEnumTemplate<bool>(name, displayName, description, defaultValue, {false, true},
    valueMeaning, valueDescription) {}
    
  virtual ~BoolFilterParameter() {}
};

/**
 * @}
 */

}

#endif // VOXEL_FILTER_PARAMETER_H