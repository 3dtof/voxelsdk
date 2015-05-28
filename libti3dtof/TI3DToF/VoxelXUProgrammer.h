/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXELXU_PROGRAMMER_H
#define VOXEL_TI_VOXELXU_PROGRAMMER_H

#include "UVCXU.h"
#include "VoxelProgrammerBase.h"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT VoxelXUProgrammer: public VoxelProgrammerBase
{
protected:
  UVCXUPtr _xu;

  const int _XU_ID = 3;
  
  uint8_t _majorVersion, _minorVersion;
  
  enum Control
  {
    CONTROL_WRITE_REGISTER_3 = 1, // Write 3 bytes
    CONTROL_SET_READ_REGISTER = 2, // Set read register
    CONTROL_READ_REGISTER_3 = 3, // Read 3 bytes
    CONTROL_WRITE_REGISTER_1 = 4, // Write 1 byte
    CONTROL_READ_REGISTER_1 = 5, // Read 1 byte
    CONTROL_REBOOT_FW_MODE = 6, // Reboot to firmware mode
    CONTROL_GET_VERSION = 7, // Get 2 bytes (major.minor) version
    CONTROL_WRITE_REGISTER_2 = 8, // Write 2 bytes
    CONTROL_READ_REGISTER_2 = 9, // Write 2 bytes
    
  };
  
  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length);
  
public:
  VoxelXUProgrammer(const SlaveAddressToByteMap &map, DevicePtr device);
  
  virtual bool isInitialized() const;
  
  void init();
  virtual bool reset();
  
  virtual ~VoxelXUProgrammer() {}
};

}
}

#endif // VOXEL_TI_VOXELXU_PROGRAMMER_H
