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
#include "Logger.h"

#include <type_traits>

namespace Voxel
{
  
class ParameterDMLParser;

class VOXEL_EXPORT Parameter
{
public:
  enum IOType
  {
    IO_READ_ONLY = 0,
    IO_READ_WRITE
  };
  
protected:
  uint32_t _address, _mask;
  uint8_t _msb, _lsb, _registerLength;
  // This is to do @_address <- (@_address & _mask) | (_value << _lsb)
  
  IOType _ioType;
  
  String _name;
  String _displayName;
  String _description;
  
  Vector<String> _dependencies; // Parameter values on which this parameter depends on
  
  RegisterProgrammer &_programmer;
  
  void _computeMask();
  
public:
  Parameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
            const String &displayName, const String &description, IOType ioType = IO_READ_WRITE, const Vector<String> &dependencies = {})
  : _programmer(programmer), _name(name), _displayName(displayName), _description(description), 
  _address(address), _msb(msb), _registerLength(registerLength), _lsb(lsb), _ioType(ioType), _dependencies(dependencies)
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
  
  inline IOType ioType() const { return _ioType; }
  
  inline void setName(const String &n) { _name = n; }
  inline void setAddress(uint32_t a) { _address = a; }
  
  virtual ~Parameter() {}
  
  friend class ParameterDMLParser;
};

typedef Ptr<Parameter> ParameterPtr;

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
template <typename T>
class ParameterTemplate: public Parameter
{
protected:
  T _value;
  
  virtual uint32_t _toRawValue(T value) const
  {
    return (uint32_t)value;
  }
  
  virtual T _fromRawValue(uint32_t value) const
  {
    return (T)(value);
  }
  
public:
  ParameterTemplate(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                    const T &defaultValue,
                    const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  Parameter(programmer, name, address, registerLength, msb, lsb, displayName, description, ioType, dependencies), _value(defaultValue)
  {
  }
  
  virtual bool set(const T &value)
  {
    if(_ioType == IO_READ_ONLY || !validate(value))
    {
      return false;
    }
    
    if(_programmer.setValue(*this, _toRawValue(value))) 
    { 
      _value = value;
      return true;       
    } 
    else 
      return false;
  }
  
  virtual bool validate(const T &value) const = 0;
  
  virtual bool get(T &value, bool refresh = true) const
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
        value = val;
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

template <typename T>
class EnumParameterTemplate: public ParameterTemplate<T>
{
protected:
  Vector<String> _valueMeaning;
  Vector<String> _valueDescription;
  
public:
  EnumParameterTemplate(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                const Vector<String> &valueDescription, const Vector<String> &valueMeaning, const T &defaultValue,
                const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  ParameterTemplate<T>(programmer, name, address, registerLength, msb, lsb, defaultValue, displayName, description, ioType, dependencies), _valueDescription(valueDescription), _valueMeaning(valueMeaning)
  {
  }
  
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
  inline const Vector<String> &valueMeaning() const { return _valueMeaning; }
  
  virtual ~EnumParameterTemplate() {}
};

class VOXEL_EXPORT BoolParameter : public EnumParameterTemplate<bool>
{
  virtual bool _fromRawValue(uint32_t value) const
  {
    return (value?true:false);
  }

  virtual uint32_t _toRawValue(bool value) const
  {
    return (uint32_t)value?1:0;
  }
public:
  BoolParameter(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t lsb, 
                const Vector<String> &valueDescription, const Vector<String> &valueMeaning, const bool &defaultValue,
                const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  EnumParameterTemplate<bool>(programmer, name, address, registerLength, lsb, lsb, valueDescription, valueMeaning, defaultValue, displayName, description, ioType, dependencies)
  {
  }
  
  virtual bool validate(const bool &value) const
  {
    return true; 
  }
  
  virtual ~BoolParameter() {}
};

class VOXEL_EXPORT StrobeBoolParameter : public BoolParameter
{
public:
  StrobeBoolParameter(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t registerLength, uint8_t lsb, 
                const Vector<String> &valueDescription, const Vector<String> &valueMeaning, const bool &defaultValue,
                const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  BoolParameter(programmer, name, address, registerLength, lsb, valueDescription, valueMeaning, defaultValue, displayName, description, ioType, dependencies)
  {
  }
  
  virtual bool get(bool &value, bool refresh = true) const
  {
    return BoolParameter::get(value, true); // ignore the refresh set by user and force it to true
  }
  
  virtual ~StrobeBoolParameter() {}
};

class VOXEL_EXPORT EnumParameter : public EnumParameterTemplate<int>
{
protected:
  Vector<int> _allowedValues;
  
public:
  EnumParameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                const Vector<int> &allowedValues, const Vector<String> valueDescription, const Vector<String> &valueMeaning, const int &defaultValue,
                const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  EnumParameterTemplate<int>(programmer, name, address, registerLength, msb, lsb, valueDescription, valueMeaning, defaultValue, displayName, description, ioType, dependencies), 
  _allowedValues(allowedValues)
  {
  }
  
  inline const Vector<int> &allowedValues() const { return _allowedValues; }
  
  virtual bool validate(const int &value) const
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
class RangeParameterTemplate : public ParameterTemplate<T>
{
protected:
  T _lowerLimit, _upperLimit;
  
  String _unit;

public:
  RangeParameterTemplate(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                   const T &lowerLimit, const T &upperLimit, const T &defaultValue,
                   const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  ParameterTemplate<T>(programmer, name, address, registerLength, msb, lsb, defaultValue, displayName, description, ioType, dependencies), 
  _lowerLimit(lowerLimit), _upperLimit(upperLimit), _unit(unit)
  {
  }
  
  const String &unit() const { return _unit; }
  
  const T &lowerLimit() const { return _lowerLimit; }
  const T &upperLimit() const { return _upperLimit; }
  
  virtual bool validate(const T &value) const
  {
    return !(value < _lowerLimit || value > _upperLimit); 
  }
  
  virtual ~RangeParameterTemplate() {}
};

class VOXEL_EXPORT IntegerParameter : public RangeParameterTemplate<int>
{
protected:
  virtual uint32_t _toRawValue(int value) const
  {
    if(value < 0) // negative?
    {
      return ((uint32_t)value & ((1 << (_msb - _lsb + 1)) - 1)); // remove sign extension
    }
    else
      return (uint32_t)value;
  }
  
  virtual int _fromRawValue(uint32_t value) const
  {
    if(value & (1 << (_msb - _lsb))) // negative?
      return (value | ((uint32_t)(-1) - ((1 << (_msb - _lsb + 1)) - 1))); // extend sign
    else
      return (int)value;
  }
  
public:
  IntegerParameter(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                 int lowerLimit, int upperLimit, const int &defaultValue,
                 const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  RangeParameterTemplate<int>(programmer, name, unit, address, registerLength, msb, lsb, lowerLimit, upperLimit, defaultValue, displayName, description, ioType, dependencies)
  {
  }
  
  virtual ~IntegerParameter() {}
};

typedef RangeParameterTemplate<uint> UnsignedIntegerParameter;

class VOXEL_EXPORT FloatParameter : public RangeParameterTemplate<float>
{
protected:
  virtual float _fromRawValue(uint32_t value) const
  {
    float v;
    v = (float)value/(1 << (msb() - lsb() + 1)); // normalized value
    
    if(v > 1.0f) v = 1.0f;
    if(v < 0.0f) v = 0.0f;
    return v;
  }
  
  virtual uint32_t _toRawValue(float value) const
  {
    uint32_t maxValue = (1 << (msb() - lsb() + 1));
    uint32_t v = (uint32_t)value*maxValue; // normalized value
    
    if(v > maxValue) v = maxValue;
    if(v < 0) v = 0;
    return v;
  }
  
  
public:
  FloatParameter(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t registerLength, uint8_t msb, uint8_t lsb, 
                         float lowerLimit, float upperLimit, const float &defaultValue,
                 const String &displayName, const String &description, Parameter::IOType ioType = Parameter::IO_READ_WRITE, const Vector<String> &dependencies = {}):
  RangeParameterTemplate<float>(programmer, name, unit, address, registerLength, msb, lsb, lowerLimit, upperLimit, defaultValue, displayName, description, ioType, dependencies)
  {
  }
  
  virtual ~FloatParameter() {}
};

}

#endif // PARAMETER_H
