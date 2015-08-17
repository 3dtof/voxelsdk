/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 */

#include "RTS5825CameraFactory.h"
#include <Logger.h>

#include "RTS5825Camera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
RTS5825CameraFactory::RTS5825CameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(RTS5825CAMERA_VENDOR_ID, RTS5825CAMERA_PRODUCT_ID1, "")),
  });
}

DepthCameraPtr RTS5825CameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == RTS5825CAMERA_VENDOR_ID && 
      (d.productID() == RTS5825CAMERA_PRODUCT_ID1))
    {
      return DepthCameraPtr(new RTS5825Camera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new RTS5825CameraFactory("RTS5825Camera"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}