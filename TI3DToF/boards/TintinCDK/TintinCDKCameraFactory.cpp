/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "TintinCDKCameraFactory.h"
#include <Logger.h>

#include "TintinCDKCamera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
TintinCDKCameraFactory::TintinCDKCameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_BULK, "")),
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_UVC, "")),
  });
}

DepthCameraPtr TintinCDKCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == TINTIN_CDK_VENDOR_ID && 
      (d.productID() == TINTIN_CDK_PRODUCT_BULK || d.productID() == TINTIN_CDK_PRODUCT_UVC))
    {
      return DepthCameraPtr(new TintinCDKCamera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new TintinCDKCameraFactory("TintinCDK"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}