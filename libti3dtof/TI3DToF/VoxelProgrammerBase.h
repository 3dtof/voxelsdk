/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXEL_PROGRAMMER_BASE_H
#define VOXEL_TI_VOXEL_PROGRAMMER_BASE_H

#include "RegisterProgrammer.h"
#include "Device.h"

#include "TI3DToFExports.h"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT VoxelProgrammerBase: public RegisterProgrammer
{
public:
  typedef Map<uint, uint> SlaveAddressToByteMap;
  
protected:
  DevicePtr _device;
  
  mutable Mutex _mutex;
  
  SlaveAddressToByteMap _slaveAddressToByteMap;
  
  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const = 0;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length) = 0;
  
public:
  VoxelProgrammerBase(const SlaveAddressToByteMap &map, DevicePtr device);
  
  virtual bool readRegister(uint32_t address, uint32_t &value) const;
  virtual bool writeRegister(uint32_t address, uint32_t value);
  
  // getValue() and setValue() are internally called readRegister() and writeRegister() which are thread-safe. So, no mutex locking in getValue() and setValue()
  virtual bool getValue(const Parameter &param, uint32_t &value) const;
  virtual bool setValue(const Parameter &param, uint32_t value, bool writeOnly = false);
  
  virtual ~VoxelProgrammerBase() {}
};

}
}

#endif // VOXEL_TI_VOXELXU_PROGRAMMER_H
