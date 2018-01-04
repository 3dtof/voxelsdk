/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_VoxelVader_H
#define VOXEL_TI_VoxelVader_H

#include <ToFCalculusCamera.h>
#include <Downloader.h>
#include "UVCXU.h"


#define VOXELVADER_VENDOR_ID 0x0451U
#define VOXELVADER_PRODUCT_ID 0x9108U

#define ILLUM_VOLTAGE "illum_volt" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class VoxelVader: public ToFCalculusCamera
{
protected:
  Ptr<Downloader> _downloader;
  UVCXUPtr _xu;
  const int _XU_ID = 3;

  enum FlashControl
  {
    CONTROL_READ_WRITE_REGISTER_START = 7, //Start address for reading/writing
    CONTROL_READ_WRITE_REGISTER_32 = 8, //Read/Write 32 bytes
    CONTROL_ERASE_DATA = 9 //Erase a sector before write 
  };
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  
  virtual bool _initStartParams();
  virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
  
public:
  VoxelVader(DevicePtr device);
  
  virtual ~VoxelVader() {}

  virtual bool setSerialNumber(const String& serialNumber);
  virtual bool getSerialNumber(String &serialNumber) const;
};

}
}

#endif // VOXEL_TI_VoxelVader_H
