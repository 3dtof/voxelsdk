/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VOXEL_USB_PROGRAMMER_H
#define VOXEL_TI_VOXEL_USB_PROGRAMMER_H

#include "USBIO.h"
#include "VoxelProgrammerBase.h"

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT VoxelUSBProgrammer: public VoxelProgrammerBase
{
public:
  struct RequestParams
  {
    uint8_t readRequestCode, writeRequestCode;
    uint8_t leftShiftBits;
  };
  
  typedef Map<uint, RequestParams> SlaveAddressToRequestParamsMap;
protected:
  USBIOPtr _usbIO;
  SlaveAddressToRequestParamsMap _slaveAddressToRequestParamsMap;

  virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const;
  virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length);
  
public:
  VoxelUSBProgrammer(const SlaveAddressToByteMap &map, const SlaveAddressToRequestParamsMap &slaveAddressToRequestParamsMap, USBIOPtr &usbIO, DevicePtr device);
  
  virtual bool isInitialized() const;
  
  virtual bool reset();
  
  inline USBIOPtr &getUSBIO() { return _usbIO; }
  
  virtual ~VoxelUSBProgrammer() {}
};

}
}

#endif // VOXEL_TI_VOXELXU_PROGRAMMER_H
