/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PARAMETER_H
#define VOXEL_PARAMETER_H

#include <stdint.h>

#include "Common.h"

#include "RegisterProgrammer.h"

namespace Voxel
{
  
class RegisterProgrammer;

class Parameter
{
protected:
  uint32_t _address, _mask;
  uint8_t _msb, _lsb, _registerLength;
  // This is to do @_address <- (@_address & _mask) | (_value << _lsb)
  
  String _name;
  String _displayName;
  String _description;
  
  RegisterProgrammer &_programmer;
  
  void _computeMask();
  
public:
  Parameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
            const String &displayName, const String &description)
  : _programmer(programmer), _name(name), _displayName(displayName), _description(description), _address(address), _msb(msb), _registerLength(registerLength), _lsb(lsb) 
  {
    _computeMask();
  }
  
  inline const String &name() const { return _name; }
  inline const String &displayName() const { return _displayName; }
  inline const String &description() const { return _description; }
  inline uint32_t address() const { return _address; }
  inline uint8_t msb() const { return _msb; }
  inline uint32_t mask() const { return _mask; }
  inline uint8_t lsb() const { return _lsb; }
  
  virtual ~Parameter() {}
};

typedef Ptr<Parameter> ParameterPtr;

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
template <typename T>
class ParameterTemplate: public Parameter
{
protected:
  T _value;
  
  virtual uint32_t _toRawValue(T value)
  {
    return (uint32_t)value;
  }
  
  virtual T _fromRawValue(uint32_t value)
  {
    return (T)value;
  }
  
public:
  ParameterTemplate(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                const String &displayName, const String &description):
  Parameter(programmer, name, address, registerLength, msb, lsb, displayName, description)
  {
  }
  
  virtual bool set(T value)
  {
    if(!validate(value))
      return false;
    
    if(_programmer.setValue(*this, _toRawValue(value))) 
    { 
      _value = value;
      return true;       
    } 
    else 
      return false;
  }
  
  virtual bool validate(T value) = 0;
  
  virtual bool get(T &value, bool refresh = false)
  {
    if(!refresh)
    {
      value = _value;
      return true;
    }
    
    uint32_t v;
    T val;
    if(!_programmer.getValue(*this, v))
    {
      value = _value;
      return false;
    }
    else
    {
      val = _fromRawValue(v);
      if(validate(val))
      {
        value = _value = val;
        return true;
      }
      else
      {
        value = _value;
        return false;
      }
    }
  }
  
  virtual ~ParameterTemplate() {}
};


class BoolParameter: public ParameterTemplate<bool>
{
protected:
  Vector<String> _valueDescription;

  BoolParameter(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                const Vector<String> &valueDescription,
                const String &displayName, const String &description):
  ParameterTemplate<bool>(programmer, name, address, registerLength, msb, lsb, displayName, description), _valueDescription(valueDescription) 
  {
  }
  
  virtual bool validate (bool value) { return true; }
  
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
              
  virtual ~BoolParameter() {}
};

class EnumParameter: public ParameterTemplate<int>
{
protected:
  Vector<int> _allowedValues;
  Vector<String> _valueDescription;
  
public:
  EnumParameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                const Vector<int> &allowedValues, const Vector<String> valueDescription,
                const String &displayName, const String &description):
  ParameterTemplate<int>(programmer, name, address, registerLength, msb, lsb, displayName, description), 
  _allowedValues(allowedValues), _valueDescription(valueDescription)
  {
  }
  
  inline const Vector<int> &allowedValues() const { return _allowedValues; }
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
  
  virtual bool validate(int value)
  {
    bool allowed = false;
    for(auto a : _allowedValues)
      if(value == a)
      {
        allowed = true;
        break;
      }
      
      return allowed;
  }
  
  virtual ~EnumParameter() {}
};



template<typename T>
class RangeParameterTemplate: public ParameterTemplate<T>
{
protected:
  T _lowerLimit, _upperLimit;
  
  String _unit;

public:
  RangeParameterTemplate(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                   T lowerLimit, T upperLimit,
                   const String &displayName, const String &description):
  ParameterTemplate<T>(programmer, name, address, registerLength, msb, lsb, displayName, description), 
  _lowerLimit(lowerLimit), _upperLimit(upperLimit), _unit(unit)
  {
  }
  
  const String &unit() const { return _unit; }
  
  virtual bool validate (T value) { return !(value < _lowerLimit or value > _upperLimit); }
  
  virtual ~RangeParameterTemplate() {}
};

typedef RangeParameterTemplate<int> IntegerParameter;

class FloatParameter: public RangeParameterTemplate<float>
{
protected:
  virtual float _fromRawValue(uint32_t value)
  {
    float v;
    v = value/(1 << (msb() - lsb() + 1)); // normalized value
    
    if(v > 1.0f) v = 1.0f;
    if(v < 0.0f) v = 0.0f;
    return v;
  }
  
  virtual uint32_t _toRawValue(float value)
  {
    uint32_t maxValue = (1 << (msb() - lsb() + 1));
    uint32_t v = (uint32_t)value*maxValue; // normalized value
    
    if(v > maxValue) v = maxValue;
    if(v < 0) v = 0;
    return v;
  }
  
  
public:
  FloatParameter(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                         float lowerLimit, float upperLimit,
                         const String &displayName, const String &description):
  RangeParameterTemplate<float>(programmer, name, unit, address, registerLength, msb, lsb, lowerLimit, upperLimit, displayName, description)
  {
  }
  
  virtual ~FloatParameter() {}
};

}

#endif // PARAMETER_H
