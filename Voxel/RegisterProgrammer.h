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
  virtual bool isInitialized() const = 0;
  
  virtual bool setValue(const Parameter &param, uint32_t value) = 0;
  virtual bool getValue(const Parameter &param, uint32_t &value) const = 0;
  
  virtual bool readRegister(uint32_t address, uint32_t &value) const = 0;
  virtual bool writeRegister(uint32_t address, uint32_t value) = 0;
  
  virtual bool reset() = 0;
  
  virtual ~RegisterProgrammer() {}
};
  
}

#endif