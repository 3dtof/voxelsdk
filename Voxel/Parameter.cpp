/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Parameter.h"

namespace Voxel
{
 
bool BoolParameter::validate(bool value)
{
  return true;
}
  
bool BoolParameter::set(bool value)
{
  if(!validate(value))
    return false;
  
  if(_programmer.set(*this, value)) 
  { 
    _value = value; 
    return true;       
  } 
  else 
    return false;
}

bool BoolParameter::get()
{
  return _value;
}


bool IntegerParameter::validate(int value)
{
  return (value < _lowerLimit or value > _upperLimit);
}

bool IntegerParameter::set(int value) 
{ 
  if(!validate(value))
    return false;
  
  if(_programmer.set(*this, value)) 
  { 
    _value = value; 
    return true;       
  } 
  else 
    return false;
}

int IntegerParameter::get()
{
  return _value;
}


bool FloatParameter::validate(float value)
{
  return (value < _lowerLimit or value > _upperLimit);
}

bool FloatParameter::set(float value)
{ 
  if(!validate(value)) 
    return false;  
  
  if(_programmer.set(*this, value)) 
  { 
    _value = value; 
    return true;       
  } 
  else 
    return false;
}

float FloatParameter::get()
{
  return _value;
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
  
  if(_programmer.set(*this, value)) 
  { 
    _value = value; 
    return true;       
  }
  else
    return false;
}

int EnumParameter::get()
{
  return _value;
}


}