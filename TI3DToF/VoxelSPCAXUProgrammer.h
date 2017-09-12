/*
 * VoxelSPCAXUProgrammer.h
 *
 *  Created on: Jun 7, 2017
 *      Author: a0229967
 */

#ifndef TI3DTOF_VOXELSPCAXUPROGRAMMER_H_
#define TI3DTOF_VOXELSPCAXUPROGRAMMER_H_

#include "VoxelXUProgrammer.h"

namespace Voxel
{
namespace TI
{

class TI3DTOF_EXPORT VoxelSPCAXUProgrammer : public VoxelProgrammerBase
{
protected:
	UVCXUPtr _xu;

	const int _XU_ID = 3;

	enum Control
	{
		CONTROL_WRITE_REGISTER_2 = 5, //Write 2 bytes
		CONTROL_READ_WRITE_REGISTER_3 = 6, //Read/Write 3 bytes
		CONTROL_READ_WRITE_REGISTER_16 = 7 //Read/Write 16 bytes

	};
	virtual bool _readRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t &value, uint8_t length) const;
	virtual bool _writeRegister(uint16_t slaveAddress, uint16_t registerAddress, uint32_t value, uint8_t length);


public:
	VoxelSPCAXUProgrammer(const SlaveAddressToByteMap &map, DevicePtr device);
	  virtual bool isInitialized() const;

	  void init();
	  virtual ~VoxelSPCAXUProgrammer() {}
	  virtual bool reset();
};
}
}




#endif /* TI3DTOF_VOXELSPCAXUPROGRAMMER_H_ */
