/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelXUProgrammer.h"
#include "Parameter.h"
#include "Logger.h"

namespace Voxel
{
  
namespace TI
{
  
bool VoxelXUProgrammer::isInitialized() const
{
  return (bool)_xu;
}

  
VoxelXUProgrammer::VoxelXUProgrammer(DevicePtr device)
{
  if(device->interface() != Device::USB)
  {
    logger(ERROR) << "VoxelXUProgrammer: Invalid device." << std::endl;
    return;
  }
  
  _device = device;
  
  _xu = UVCXUPtr(new UVCXU(device, _XU_ID));
  
  uint8_t data[4];
  if(!_xu->getControl(CONTROL_GET_VERSION, arraySize(data), data))
  {
    logger(WARNING) << "VoxelXUProgrammer: Could not get XU version." << std::endl;
    return;
  }
  _minorVersion = data[0];
  _majorVersion = data[1];
  
  logger(INFO) << "VoxelXUProgrammer: XU controller version " << (int)_majorVersion << "." << (int)_minorVersion << std::endl;
}

bool VoxelXUProgrammer::readRegister(uint32_t address, uint32_t &value) const
{
  if(!isInitialized())
  {
    logger(ERROR) << "VoxelXUProgrammer: Not initialized." << std::endl;
    return false;
  }
  
  uint8_t addr[2];
  
  addr[0] = (address & 0xFF00) >> 8; // I2C slave address
  addr[1] = (address & 0xFF); // I2C register address
  
  if(!_xu->setControl(CONTROL_SET_READ_REGISTER, arraySize(addr), addr))
  {
    logger(ERROR) << "VoxelXUProgrammer: Could not set read register address 0x" << std::hex << address << std::endl;
    return false;
  }
  
  if(addr[0] == 0x2D) // TPS
  {
    uint8_t data[1];
    data[0] = 0x0;
    
    if(!_xu->getControl(CONTROL_READ_REGISTER_1, arraySize(data), data))
    {
      logger(ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << address << std::endl;
      return false;
    }
    
    value = data[0];
  }
  else // ToF
  {
    uint8_t data[3];
    data[0] = 0x0;
    data[1] = 0x0;
    data[2] = 0x0;
    if(!_xu->getControl(CONTROL_READ_REGISTER_3, arraySize(data), data))
    {
      logger(ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << address << std::endl;
      return false;
    }
    
    value = data[0] + (data[1] << 8) + (data[2] << 16);
  }
  
  return true;
}

bool VoxelXUProgrammer::writeRegister(uint32_t address, uint32_t value)
{
  if(!isInitialized())
  {
    logger(ERROR) << "VoxelXUProgrammer: Not initialized." << std::endl;
    return false;
  }
  
  uint8_t i2cSlaveAddress, i2cRegisterAddress;
  
  i2cSlaveAddress = (address & 0xFF00) >> 8;
  i2cRegisterAddress = (address & 0xFF);
  
  if(i2cSlaveAddress == 0x2D) // TPS 
  {
    uint8_t data[3];
    data[0] = i2cSlaveAddress;
    data[1] = i2cRegisterAddress;
    data[2] = value & 0xFF;
    
    if(!_xu->setControl(CONTROL_WRITE_REGISTER_1, arraySize(data), data))
    {
      logger(ERROR) << "VoxelXUProgrammer: Could not write register for address 0x" << std::hex << address << std::endl;
      return false;
    }
  }
  else // ToF
  {
    uint8_t data[5];
    data[0] = i2cSlaveAddress;
    data[1] = i2cRegisterAddress;
    data[2] = (value & 0xFF);
    data[3] = (value >> 8) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    if(!_xu->setControl(CONTROL_WRITE_REGISTER_3, arraySize(data), data))
    {
      logger(ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << address << std::endl;
      return false;
    }
  }
  return true;
}

bool VoxelXUProgrammer::getValue(const Voxel::Parameter &param, uint32_t &value) const
{
  uint32_t registerValue;
  
  if(param.mask() == -1)
    return false; // No way to read value
  
  if(!readRegister(param.address(), registerValue))
    return false;
  
  value = (registerValue & ~param.mask()) >> param.lsb();
  return true;
}

bool VoxelXUProgrammer::setValue(const Parameter &param, uint32_t value)
{
  uint32_t registerValue;
  
  if(param.mask() == -1)
    return true; // Fictitiously accept set
  
  if(!readRegister(param.address(), registerValue))
    return false;
  
  registerValue = ((registerValue & param.mask()) | (value << param.lsb()));
  
  return writeRegister(param.address(), registerValue);
}

  
}
}