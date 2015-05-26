/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_TI_CALCULUSCDKCAMERA_H
#define VOXEL_TI_CALCULUSCDKCAMERA_H

#include <ToFCalculusCamera.h>
#include <Downloader.h>

#include "TI3DToFExports.h"

#define CALCULUS_CDK_VENDOR_ID 0x0451U
#define CALCULUS_CDK_PRODUCT_ID 0x9107U

#define PIXELVDD_VOLTAGE "pixel_vdd" // Illumination voltage
#define MIX_VOLTAGE "mix_volt" // Mixing voltage

namespace Voxel
{
  
namespace TI
{

class TI3DTOF_EXPORT CalculusCDKCamera : public ToFCalculusCamera
{
protected:
  Ptr<Downloader> _downloader;
  USBIOPtr _usbIO;
  
  bool _init();
  
  virtual bool _getFieldOfView(float &fovHalfAngle) const;
  virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
  virtual bool _setStreamerFrameSize(const FrameSize &s);
  
  virtual bool _getMaximumVideoMode(VideoMode &videoMode) const;
  virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
  
  virtual bool _initStartParams();
  
public:
  CalculusCDKCamera(DevicePtr device);
  
  virtual ~CalculusCDKCamera() {}
};

}
}

#endif // VOXEL_TI_CALCULUSCDKCAMERA_H
