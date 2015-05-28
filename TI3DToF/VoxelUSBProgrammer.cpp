/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelUSBProgrammer.h"
#include "Parameter.h"
#include "Logger.h"

namespace Voxel
{
  
namespace TI
{
  
bool VoxelUSBProgrammer::isInitialized() const
{
  return _usbIO->isInitialized();
}
  

VoxelUSBProgrammer::VoxelUSBProgrammer(const SlaveAddressToByteMap &map, const SlaveAddressToRequestParamsMap &slaveAddressToRequestParamsMap, USBIOPtr &usbIO, DevicePtr device): VoxelProgrammerBase(map, device), 
  _slaveAddressToRequestParamsMap(slaveAddressToRequestParamsMap), _usbIO(usbIO)
{
  if(device->interfaceID() != Device::USB)
  {
    logger(LOG_ERROR) << "VoxelUSBProgrammer: Invalid device." << std::endl;
    return;
  }
}

bool VoxelUSBProgrammer::_readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const
{
  auto x = _slaveAddressToRequestParamsMap.find(slaveAddress);
  
  if(x == _slaveAddressToRequestParamsMap.end())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Unknown slave address 0x" << std::hex << slaveAddress << std::endl;
    return false;
  }
  
  const RequestParams &y = x->second;
  
  if(length == 1) // TPS -- addr[0] == 0x2D
  {
    uint8_t data[1];
    data[0] = 0x0;
    
    if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
        y.readRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 1))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
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
    
    if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      y.readRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 3))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
    
    value = data[0] + (data[1] << 8) + (data[2] << 16);
  }
  else if(length == 2)
  {
    uint8_t data[2];
    data[0] = 0x0;
    data[1] = 0x0;
    
    if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      y.readRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 2))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
    
    value = data[0] + (data[1] << 8);
  }
  else
  {
    logger(LOG_ERROR) << "VoxelUSBProgrammer: Don't know how to read '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  
  return true;
}

bool VoxelUSBProgrammer::_writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length)
{
  auto x = _slaveAddressToRequestParamsMap.find(slaveAddress);
  
  if(x == _slaveAddressToRequestParamsMap.end())
  {
    logger(LOG_ERROR) << "VoxelProgrammerBase: Unknown slave address 0x" << std::hex << slaveAddress << std::endl;
    return false;
  }
  
  RequestParams &y = x->second;
  
  if(length == 1) // TPS  -- slaveAddress == 0x2D
  {
    uint8_t data[1];
    data[0] = value & 0xFF;
    
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      y.writeRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 1))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else if(length == 3) // ToF
  {
    uint8_t data[3];
    data[0] = (value & 0xFF);
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      y.writeRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 3))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else if(length == 2)
  {
    uint8_t data[2];
    data[0] = (value & 0xFF);
    data[1] = (value >> 8) & 0xFF;
    
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      y.writeRequestCode, slaveAddress << y.leftShiftBits, registerAddress << y.leftShiftBits, data, 2))
    {
      logger(LOG_ERROR) << "VoxelUSBProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
  }
  else
  {
    logger(LOG_ERROR) << "VoxelUSBProgrammer: Don't know how to write '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  
  return true;
}

bool VoxelUSBProgrammer::reset()
{
  if(!isInitialized())
    return false;
  
  logger(LOG_WARNING) << "VoxelUSBProgrammer: Don't know how to reset the device." << std::endl;
  
  return true;
}


  
}
}