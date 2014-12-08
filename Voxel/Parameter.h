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
  uint8_t _msb;
  uint8_t _lsb;
  // This is to do @_address <- (@_address & _mask) | (_value << _lsb)
  
  String _name;
  String _displayName;
  String _description;
  
  RegisterProgrammer &_programmer;
  
public:
  Parameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t msb, uint32_t mask, uint8_t lsb, 
            const String &displayName, const String &description)
  : _programmer(programmer), _name(name), _displayName(displayName), _description(description), _address(address), _msb(msb), _mask(mask), _lsb(lsb) {}
  
  inline const String &name() const { return _name; }
  inline const String &displayName() const { return _displayName; }
  inline const String &description() const { return _description; }
  inline uint32_t address() const { return _address; }
  inline uint8_t msb() const { return _msb; }
  inline uint32_t mask() const { return _mask; }
  inline uint8_t lsb() const { return _lsb; }
  
  virtual ~Parameter() {}
  
  friend class RegisterProgrammer;
};

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
class BoolParameter: public Parameter
{
protected:
  bool _value;
  
  Vector<String> _valueDescription;
  
  virtual uint32_t _toRawValue(bool value);
  virtual bool _fromRawValue(uint32_t value);
  
public:
  BoolParameter(RegisterProgrammer &programmer, const String &name,  uint32_t address, uint8_t msb, uint32_t mask, uint8_t lsb, 
                const Vector<String> &valueDescription,
                const String &displayName, const String &description):
  Parameter(programmer, name, address, msb, mask, lsb, displayName, description), _valueDescription(valueDescription) 
  {
  }
  
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
  
  virtual bool set(bool value);
  virtual bool validate(bool value);
  
  virtual bool get(bool refresh = false);
  
  virtual ~BoolParameter() {}
  
  friend class RegisterProgrammer;
};

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
class IntegerParameter: public Parameter
{
protected:
  int _value;
  int _lowerLimit, _upperLimit;
  
  String _unit;
  
  virtual uint32_t _toRawValue(int value);
  virtual int _fromRawValue(uint32_t value);
  
public:
  IntegerParameter(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t msb, uint32_t mask, uint8_t lsb, 
                   int lowerLimit, int upperLimit,
                   const String &displayName, const String &description):
  Parameter(programmer, name, address, msb, mask, lsb, displayName, description), 
  _lowerLimit(lowerLimit), _upperLimit(upperLimit), _unit(unit)
  {
  }
  
  const String &unit() const { return _unit; }
  
  virtual bool set(int value);
  virtual bool validate(int value);
  
  virtual int get(bool refresh = false);
  
  virtual ~IntegerParameter() {}
  
  friend class RegisterProgrammer;
};

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
class FloatParameter: public Parameter
{
protected:
  float _value;
  float _lowerLimit, _upperLimit;
  String _unit;
  
  virtual uint32_t _toRawValue(float value);
  virtual float _fromRawValue(uint32_t value);
  
public:
  FloatParameter(RegisterProgrammer &programmer, const String &name, const String &unit, uint32_t address, uint8_t msb, uint32_t mask, uint8_t lsb, 
                 float lowerLimit, float upperLimit,
                 const String &displayName, const String &description):
  Parameter(programmer, name, address, msb, mask, lsb, displayName, description), 
  _lowerLimit(lowerLimit), _upperLimit(upperLimit), _unit(unit)
  {
  }
  
  const String &unit() const { return _unit; }
  
  virtual bool set(float value);
  virtual bool validate(float value);
  
  virtual float get(bool refresh = false);
  
  virtual ~FloatParameter() {}
  
  friend class RegisterProgrammer;
};

// NOTE: _value is not initialized and need to be manually done from outside via set() or get(true)
class EnumParameter: public Parameter
{
protected:
  int _value;
  Vector<int> _allowedValues;
  Vector<String> _valueDescription;
  
  virtual uint32_t _toRawValue(int value);
  virtual int _fromRawValue(uint32_t value);
  
public:
  EnumParameter(RegisterProgrammer &programmer, const String &name, uint32_t address, uint8_t msb, uint32_t mask, uint8_t lsb, 
                const Vector<int> &allowedValues, const Vector<String> valueDescription,
                const String &displayName, const String &description):
  Parameter(programmer, name, address, msb, mask, lsb, displayName, description), 
  _allowedValues(allowedValues), _valueDescription(valueDescription)
  {
  }
  
  inline const Vector<int> &allowedValues() const { return _allowedValues; }
  inline const Vector<String> &valueDescription() const { return _valueDescription; }
  
  virtual bool set(int value);
  virtual bool validate(int value);
  
  virtual int get(bool refresh = false);
  
  virtual ~EnumParameter() {}
  
  friend class RegisterProgrammer;
};

}

#endif // PARAMETER_H
