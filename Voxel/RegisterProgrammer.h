/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#ifndef VOXEL_REGISTER_PROGRAMMER_H
#define VOXEL_REGISTER_PROGRAMMER_H

#include "Common.h"
#include <stdint.h>

namespace Voxel
{
  
class BoolParameter;
class IntegerParameter;
class FloatParameter;
class EnumParameter;
class Parameter;
  
class RegisterProgrammer
{
protected:
  virtual bool _setValue(Parameter &param, uint32_t value) = 0;
  virtual bool _getValue(Parameter &param, uint32_t &value) = 0;
  
public:
  virtual bool set(BoolParameter &param, bool value);
  virtual bool set(IntegerParameter &param, int value);
  virtual bool set(FloatParameter &param, float value);
  virtual bool set(EnumParameter &param, int value);
  
  virtual bool get(BoolParameter &param);
  virtual bool get(IntegerParameter &param);
  virtual bool get(FloatParameter &param);
  virtual bool get(EnumParameter &param);
  
  virtual bool readRegister(uint32_t address, uint32_t &value) = 0;
  virtual bool writeRegister(uint32_t address, uint32_t value) = 0;
};
  
}

#endif