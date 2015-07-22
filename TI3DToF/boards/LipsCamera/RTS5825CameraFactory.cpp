/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "RTS5825CameraFactory.h"
#include <Logger.h>

#include "RTS5825Camera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
LipsCameraFactory::LipsCameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(LIPS_CAMERA_VENDOR_ID, LIPS_CAMERA_PRODUCT_ID1, "")),
  });
}

DepthCameraPtr LipsCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == LIPS_CAMERA_VENDOR_ID && 
      (d.productID() == LIPS_CAMERA_PRODUCT_ID1))
    {
      return DepthCameraPtr(new LipsCamera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new LipsCameraFactory("RTS5825Camera"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}