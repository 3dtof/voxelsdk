/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelProgrammerBase.h"
#include "Parameter.h"
#include "Logger.h"

namespace Voxel
{
  
namespace TI
{
  
VoxelProgrammerBase::VoxelProgrammerBase(const SlaveAddressToByteMap &map, DevicePtr device): _slaveAddressToByteMap(map)
{
  _device = device;
}

bool VoxelProgrammerBase::readRegister(uint32_t address, uint32_t &value) const
{
  //LogLevelChanger _(LOG_DEBUG);
  
  Lock<Mutex> _(_mutex);
  
  if(!isInitialized())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Not initialized." << std::endl;
    return false;
  }
  
  uint16_t slaveAddress, registerAddress;
  
  slaveAddress = (address & 0xFF00) >> 8; // I2C slave address
  registerAddress = (address & 0xFF); // I2C register address
  
  auto x = _slaveAddressToByteMap.find(slaveAddress);
  
  if(x == _slaveAddressToByteMap.end())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Unknown slave address 0x" << std::hex << address << std::endl;
    return false;
  }
  
  uint byteCount = x->second;
  
  if(!_readRegister(slaveAddress, registerAddress, value, byteCount))
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Could not read register for address 0x" << std::hex << address  << std::dec << std::endl;
    return false;
  }
  
  logger(LOG_DEBUG) << "VoxelProgrammerBase: register read @0x" << std::hex << address << " = " << value << std::dec << std::endl;
  
  return true;
}

bool VoxelProgrammerBase::writeRegister(uint32_t address, uint32_t value)
{
  //LogLevelChanger _(LOG_DEBUG);
  Lock<Mutex> _(_mutex);
  
  if(!isInitialized())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Not initialized." << std::endl;
    return false;
  }
  
  uint16_t slaveAddress, registerAddress;
  
  slaveAddress = (address & 0xFF00) >> 8;
  registerAddress = (address & 0xFF);
  
  auto x = _slaveAddressToByteMap.find(slaveAddress);
  
  if(x == _slaveAddressToByteMap.end())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Unknown slave address 0x" << std::hex << slaveAddress << std::endl;
    return false;
  }
  
  uint byteCount = x->second;
  
  if(!_writeRegister(slaveAddress, registerAddress, value, byteCount))
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Could not read register for address 0x" << std::hex << address  << std::dec << std::endl;
    return false;
  }
  
  logger(LOG_DEBUG) << "VoxelProgrammerBase: register write @0x" << std::hex << address << " = " << value  << std::dec << std::endl;
  
  return true;
}

bool VoxelProgrammerBase::getValue(const Voxel::Parameter &param, uint32_t &value) const
{
  uint32_t registerValue;
  
  if(param.mask() == -1)
    return false; // No way to read value
  
  if(!readRegister(param.address(), registerValue))
    return false;
  
  value = (registerValue & ~param.mask()) >> param.lsb();
  return true;
}

bool VoxelProgrammerBase::setValue(const Parameter &param, uint32_t value, bool writeOnly)
{
  uint32_t registerValue;
  
  if(param.mask() == -1)
    return true; // Fictitiously accept set
    
  if(!writeOnly)
  {
    if(!readRegister(param.address(), registerValue))
      return false;
  }
  else
    registerValue = 0; // Start with zero value for write-only registers
  
  registerValue = ((registerValue & param.mask()) | (value << param.lsb()));
  
  return writeRegister(param.address(), registerValue);
}
  
}
}