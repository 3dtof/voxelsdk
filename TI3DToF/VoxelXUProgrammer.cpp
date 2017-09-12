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

void VoxelXUProgrammer::init()
{
  _xu = UVCXUPtr(new UVCXU(_device, _XU_ID, 0)); // index 0 for devices with FX2

  uint8_t data[4];
  if (!_xu->getControl(CONTROL_GET_VERSION, arraySize(data), data))
  {
    logger(LOG_WARNING) << "VoxelXUProgrammer: Could not get XU version." << std::endl;
    return;
  }
  _minorVersion = data[0];
  _majorVersion = data[1];

  logger(LOG_INFO) << "VoxelXUProgrammer: XU controller version " << (int)_majorVersion << "." << (int)_minorVersion << std::endl;
}
  
VoxelXUProgrammer::VoxelXUProgrammer(const SlaveAddressToByteMap &map, DevicePtr device): VoxelProgrammerBase(map, device)
{
  if(device->interfaceID() != Device::USB)
  {
    logger(LOG_ERROR) << "VoxelXUProgrammer: Invalid device." << std::endl;
    return;
  }
  
  init();
}

bool VoxelXUProgrammer::_readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const
{
  uint8_t addr[2];
  
  addr[0] = slaveAddress;
  addr[1] = registerAddress;
  
  if(!_xu->setControl(CONTROL_SET_READ_REGISTER, arraySize(addr), addr))
  {
    logger(LOG_ERROR) << "VoxelXUProgrammer: Could not set read register address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  
  if(length == 1) // TPS -- addr[0] == 0x2D
  {
    uint8_t data[1];
    data[0] = 0x0;
    
    if(!_xu->getControl(CONTROL_READ_REGISTER_1, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
    
    value = data[0];
  }
  else if(length == 3) // ToF -- 0x58, 0x5C
  {
    uint8_t data[3];
    data[0] = 0x0;
    data[1] = 0x0;
    data[2] = 0x0;
    if(!_xu->getControl(CONTROL_READ_REGISTER_3, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
    
    value = data[0] + (data[1] << 8) + (data[2] << 16);
  }
  else if(length == 2)
  {
    uint8_t data[2];
    data[0] = 0x0;
    data[1] = 0x0;
    if(!_xu->getControl(CONTROL_READ_REGISTER_2, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
    
    value = data[0] + (data[1] << 8);
  }
  else
  {
    logger(LOG_ERROR) << "VoxelXUProgrammer: Don't know how to read '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  
  return true;
}

bool VoxelXUProgrammer::_writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length)
{
  if(length == 1) // TPS  -- slaveAddress == 0x2D
  {
    uint8_t data[3];
    data[0] = slaveAddress;
    data[1] = registerAddress;
    data[2] = value & 0xFF;
    
    if(!_xu->setControl(CONTROL_WRITE_REGISTER_1, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not write register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else if(length == 3) // ToF
  {
    uint8_t data[5];
    data[0] = slaveAddress;
    data[1] = registerAddress;
    data[2] = (value & 0xFF);
    data[3] = (value >> 8) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    if(!_xu->setControl(CONTROL_WRITE_REGISTER_3, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else if(length == 2)
  {
    uint8_t data[4];
    data[0] = slaveAddress;
    data[1] = registerAddress;
    data[2] = (value & 0xFF);
    data[3] = (value >> 8) & 0xFF;
    if(!_xu->setControl(CONTROL_WRITE_REGISTER_2, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else
  {
    logger(LOG_ERROR) << "VoxelXUProgrammer: Don't know how to write '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  
  logger(LOG_DEBUG) << "VoxelXUProgrammer: register write @0x" << std::hex << slaveAddress << registerAddress << " = " << value << std::endl;
  
  return true;
}

bool VoxelXUProgrammer::reset()
{
  if(!isInitialized())
    return false;
  
  uint8_t data[1];
  if(!_xu->setControl(CONTROL_REBOOT_FW_MODE, arraySize(data), data))
  {
    logger(LOG_ERROR) << "VoxelXUProgrammer: Could not reset the device." << std::endl;
    return false;
  }
  return true;
}


  
}
}
