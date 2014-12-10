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
  
class Parameter;
  
class RegisterProgrammer
{
public:
  virtual bool isInitialized() = 0;
  
  virtual bool setValue(Parameter &param, uint32_t value) = 0;
  virtual bool getValue(Parameter &param, uint32_t &value) = 0;
  
  virtual bool readRegister(uint32_t address, uint32_t &value) = 0;
  virtual bool writeRegister(uint32_t address, uint32_t value) = 0;
};
  
}

#endif