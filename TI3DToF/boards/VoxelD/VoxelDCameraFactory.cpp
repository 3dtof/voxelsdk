/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "VoxelDCameraFactory.h"
#include <Logger.h>

#include "VoxelDCamera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
VoxelDCameraFactory::VoxelDCameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(VOXEL_D_VENDOR_ID, VOXEL_D_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(VOXEL_D_VENDOR_ID, VOXEL_D_PRODUCT_ID2, "")),
  });
}

DepthCameraPtr VoxelDCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_D_VENDOR_ID && 
      (d.productID() == VOXEL_D_PRODUCT_ID1 || d.productID() == VOXEL_D_PRODUCT_ID2))
    {
      return DepthCameraPtr(new VoxelDCamera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new VoxelDCameraFactory("VoxelD"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}