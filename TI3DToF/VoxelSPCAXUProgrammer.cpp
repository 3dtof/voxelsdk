/*
 * VoxelSPCAXUProgrammer.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: a0229967
 */

/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelSPCAXUProgrammer.h"
#include "Parameter.h"
#include "Logger.h"

namespace Voxel
{

namespace TI
{

bool VoxelSPCAXUProgrammer::isInitialized() const
{
  return (bool)_xu;
}

void VoxelSPCAXUProgrammer::init()
{
  _xu = UVCXUPtr(new UVCXU(_device, _XU_ID, 1));	//Index 1 for the SunPlus IT chip

}

VoxelSPCAXUProgrammer::VoxelSPCAXUProgrammer(const SlaveAddressToByteMap &map, DevicePtr device): VoxelProgrammerBase(map, device)
{
	if(device->interfaceID() != Device::USB)
	  {
	    logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Invalid device." << std::endl;
	    return;
	  }

	  init();
}

bool VoxelSPCAXUProgrammer::_readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const
{
  uint8_t addr[2];

  addr[0] = registerAddress;
  addr[1] = 2*slaveAddress;

  if(!_xu->setControl(CONTROL_WRITE_REGISTER_2, arraySize(addr), addr))
  {
    logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Could not set read register address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }
  if (length <= 3)
  {
    uint8_t data[3];
    data[0] = 0x0;
    data[1] = 0x0;
    data[2] = 0x0;
    if(!_xu->getControl(CONTROL_READ_WRITE_REGISTER_3, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }

    value = data[0] + (data[1] << 8) + (data[2] << 16);
  }

  else
  {
    logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Don't know how to read '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
    return false;
  }

  return true;
}

bool VoxelSPCAXUProgrammer::_writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length)
{
 if(length == 3) // ToF
 {
    uint8_t addr[2];
    uint8_t data[3];
    addr[0] = registerAddress;
    addr[1] = 2*slaveAddress;
    data[0] = (value & 0xFF);
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    if (!_xu->setControl(CONTROL_WRITE_REGISTER_2, arraySize(addr), addr))
    {
    	logger(LOG_ERROR) <<  "VoxelSPCAXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
        return false;
    }

    if(!_xu->setControl(CONTROL_READ_WRITE_REGISTER_3, arraySize(data), data))
    {
      logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
      return false;
    }
 }

 else if(length == 1)
 {
	uint8_t addr[2];
	uint8_t data[3];
	addr[0] = registerAddress;
	addr[1] = 2*slaveAddress;
	data[0] = (value & 0xFF);
	data[1] = 0x0;
	data[2] = 0x0;

	if (!_xu->setControl(CONTROL_WRITE_REGISTER_2, arraySize(addr), addr))
	{
	  logger(LOG_ERROR) <<  "VoxelSPCAXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
	  return false;
    }

	if(!_xu->setControl(CONTROL_READ_WRITE_REGISTER_3, arraySize(data), data))
	{
	  logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Could not read register for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
	  return false;
	}

 }


 else
 {
   logger(LOG_ERROR) << "VoxelSPCAXUProgrammer: Don't know how to write '" << std::dec << length << "' bytes. Required for address 0x" << std::hex << slaveAddress << registerAddress << std::endl;
   return false;
 }

  logger(LOG_DEBUG) << "VoxelSPCAXUProgrammer: register write @0x" << std::hex << slaveAddress << registerAddress << " = " << value << std::endl;

  return true;
}

bool VoxelSPCAXUProgrammer::reset() // TODO: Don't know the control to reset the device. Ask for more details
{
  if(!isInitialized())
    return false;
  return true;
}




}
}


