/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXELXU_H
#define VOXEL_TI_VOXELXU_H

#include "RegisterProgrammer.h"
#include "Device.h"
#include "UVCXU.h"

namespace Voxel
{
  
namespace TI
{

class VoxelXUProgrammer: public RegisterProgrammer
{
protected:
  Ptr<UVCXU> _xu;
  DevicePtr _device;
  
  const int _XU_ID = 3;
  
  uint8_t _majorVersion, _minorVersion;
  
  virtual bool _getValue(Parameter &param, uint32_t &value);
  virtual bool _setValue(Parameter &param, uint32_t value);
  
  enum Control
  {
    CONTROL_WRITE_REGISTER_3 = 1, // Write 3 bytes
    CONTROL_SET_READ_REGISTER = 2, // Set read register
    CONTROL_READ_REGISTER_3 = 3, // Read 3 bytes
    CONTROL_WRITE_REGISTER_1 = 4, // Write 1 byte
    CONTROL_READ_REGISTER_1 = 5, // Read 1 byte
    CONTROL_REBOOT_FW_MODE = 6, // Reboot to firmware mode
    CONTROL_GET_VERSION = 7 // Get 2 bytes (major.minor) version
  };
  
public:
  VoxelXUProgrammer(DevicePtr device);
  
  virtual bool readRegister(uint32_t address, uint32_t &value);
  virtual bool writeRegister(uint32_t address, uint32_t value);
  
  virtual ~VoxelXUProgrammer() {}
};

}
}

#endif // VOXEL_TI_VOXELXU_H
