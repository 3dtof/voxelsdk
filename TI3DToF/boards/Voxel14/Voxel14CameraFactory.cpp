/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Voxel14CameraFactory.h"
#include <Logger.h>

#include "Voxel14Camera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
Voxel14CameraFactory::Voxel14CameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID2, "")),
  });
}

DepthCameraPtr Voxel14CameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_14_VENDOR_ID && 
      (d.productID() == VOXEL_14_PRODUCT_ID1 || d.productID() == VOXEL_14_PRODUCT_ID2))
    {
      return DepthCameraPtr(new Voxel14Camera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new Voxel14CameraFactory("Voxel14"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}