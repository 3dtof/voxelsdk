/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "RegisterProgrammer.h"
#include "Parameter.h"
#include "Logger.h"

namespace Voxel
{
  
bool RegisterProgrammer::get(EnumParameter &param)
{
  if(param.mask() == -1)
    return false;
  
  uint32_t value;
  if(_getValue(param, value))
  {
    if(param.validate((int32_t)value))
    {
      param._value = (int32_t)value;
      return true;
    }
    else
    {
      log(WARNING) << "RegisterProgrammer: read invalid value " << (int32_t)value << " for parameter '" << param.name() << std::endl;
      return false;
    }
  }
  else
    return false;
}

bool RegisterProgrammer::get(BoolParameter& param)
{
  if(param.mask() == -1)
    return false;
  
  uint32_t value;
  if(_getValue(param, value))
  {
    if(param.validate(value?true:false))
    {
      param._value = value?true:false;
      return true;
    }
    else
    {
      log(WARNING) << "RegisterProgrammer: read invalid value " << (int32_t)value << " for parameter '" << param.name() << std::endl;
      return false;
    }
    return true;
  }
  else
    return false;
}

bool RegisterProgrammer::get(IntegerParameter &param)
{
  if(param.mask() == -1)
    return false;
  
  uint32_t value;
  if(_getValue(param, value))
  {
    if(param.validate((int32_t)value))
    {
      param._value = (int32_t)value;
      return true;
    }
    else
    {
      log(WARNING) << "RegisterProgrammer: read invalid value " << (int32_t)value << " for parameter '" << param.name() << std::endl;
      return false;
    }
  }
  else
    return false;
}

bool RegisterProgrammer::get(FloatParameter &param)
{
  if(param.mask() == -1)
    return false;
  
  uint32_t value;
  float v;
  if(_getValue(param, value))
  {
    v = value/(1 << (param.msb() - param.lsb() + 1)); // normalized value
    
    if(v > 1.0f) v = 1.0f;
    if(v < 0.0f) v = 0.0f;
    
    if(param.validate(value))
    {
      param._value = value;
      return true;
    }
    else
    {
      log(WARNING) << "RegisterProgrammer: read invalid value " << value << " for parameter '" << param.name() << std::endl;
      return false;
    }
  }
  else
    return false;
}

bool RegisterProgrammer::set(EnumParameter &param, int value)
{
  if(param.mask() == -1) // If no register to set, return true
    return true;
  
  return _setValue(param, value);
}

bool RegisterProgrammer::set(IntegerParameter &param, int value)
{
  if(param.mask() == -1) // If no register to set, return true
    return true;
  
  return _setValue(param, value);
}

bool RegisterProgrammer::set(BoolParameter &param, bool value)
{
  if(param.mask() == -1) // If no register to set, return true
    return true;
  
  return _setValue(param, value?1:0);
}

bool RegisterProgrammer::set(FloatParameter &param, float value)
{
  if(param.mask() == -1) // If no register to set, return true
    return true;
  
  int maxValue = (1 << (param.msb() - param.lsb() + 1));
  uint32_t v = (int)value*maxValue; // normalized value
  
  if(v > maxValue) v = maxValue;
  if(v < 0) v = 0;
  
  return _setValue(param, v);
}

  
}