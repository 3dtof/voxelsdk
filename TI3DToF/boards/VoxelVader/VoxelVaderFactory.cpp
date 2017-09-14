/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#include "VoxelVaderFactory.h"

#include <Logger.h>
#include "VoxelVader.h"
#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
VoxelVaderFactory::VoxelVaderFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(VOXELVADER_VENDOR_ID, VOXELVADER_PRODUCT_ID, "")),
  });
}

DepthCameraPtr VoxelVaderFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXELVADER_VENDOR_ID &&
      (d.productID() == VOXELVADER_PRODUCT_ID))
    {
      return DepthCameraPtr(new VoxelVader(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new VoxelVaderFactory("VoxelVader"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}
