/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Parameter.h"

namespace Voxel
{
  
  
void Parameter::_computeMask()
{
  if(_msb < _lsb)
    _mask = -1; // fictitious register
    
  _mask = (1 << _registerLength) - 1;  
  for (auto i = _lsb; i <= _msb; i++) 
  {
    _mask -= (1 << i);
  }
}

 
bool BoolParameter::validate(bool value)
{
  return true;
}
  
bool BoolParameter::set(bool value)
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

bool BoolParameter::get(bool refresh)
{
  uint32_t v;
  bool val;
  if(!refresh || !_programmer.getValue(*this, v))
    return _value;
  else
  {
    val = _fromRawValue(v);
    if(validate(val))
      return _value = val;
    else
      return _value;
  }
}

bool BoolParameter::_fromRawValue(uint32_t value)
{
  return value?true:false;
}

uint32_t BoolParameter::_toRawValue(bool value)
{
  return value?1:0;
}

bool IntegerParameter::validate(int value)
{
  return !(value < _lowerLimit or value > _upperLimit);
}

bool IntegerParameter::set(int value) 
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

int IntegerParameter::get(bool refresh)
{
  uint32_t v;
  int val;
  if(!refresh || !_programmer.getValue(*this, v))
    return _value;
  else
  {
    val = _fromRawValue(v);
    if(validate(val))
      return _value = val;
    else
      return _value;
  }
}

int IntegerParameter::_fromRawValue(uint32_t value)
{
  return (int)value;
}

uint32_t IntegerParameter::_toRawValue(int value)
{
  return (uint32_t)value;
}

bool FloatParameter::validate(float value)
{
  return !(value < _lowerLimit or value > _upperLimit);
}

bool FloatParameter::set(float value)
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

float FloatParameter::get(bool refresh)
{
  uint32_t v;
  float val;
  if(!refresh || !_programmer.getValue(*this, v))
    return _value;
  else
  {
    val = _fromRawValue(v);
    if(validate(val))
      return _value = val;
    else
      return _value;
  }
}

float FloatParameter::_fromRawValue(uint32_t value)
{
  float v;
  v = value/(1 << (msb() - lsb() + 1)); // normalized value
  
  if(v > 1.0f) v = 1.0f;
  if(v < 0.0f) v = 0.0f;
  return v;
}

uint32_t FloatParameter::_toRawValue(float value)
{
  uint32_t maxValue = (1 << (msb() - lsb() + 1));
  uint32_t v = (uint32_t)value*maxValue; // normalized value
  
  if(v > maxValue) v = maxValue;
  if(v < 0) v = 0;
  return v;
}

bool EnumParameter::validate(int value)
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


bool EnumParameter::set(int value) 
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

int EnumParameter::get(bool refresh)
{
  uint32_t v;
  int val;
  if(!refresh || !_programmer.getValue(*this, v))
    return _value;
  else
  {
    val = _fromRawValue(v);
    if(validate(val))
      return _value = val;
    else
      return _value;
  }
}

int EnumParameter::_fromRawValue(uint32_t value)
{
  return (int)value;
}

uint32_t EnumParameter::_toRawValue(int value)
{
  return (uint32_t)value;
}

}