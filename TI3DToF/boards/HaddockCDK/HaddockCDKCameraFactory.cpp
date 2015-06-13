/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "HaddockCDKCameraFactory.h"
#include <Logger.h>

#include "HaddockCDKCamera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  
  
HaddockCDKCameraFactory::HaddockCDKCameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(HADDOCK_CDK_VENDOR_ID, HADDOCK_CDK_PRODUCT_ID, "")),
  });
}

DepthCameraPtr HaddockCDKCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == HADDOCK_CDK_VENDOR_ID && (d.productID() == HADDOCK_CDK_PRODUCT_ID))
    {
      return DepthCameraPtr(new HaddockCDKCamera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new HaddockCDKCameraFactory("HaddockCDK"));
}

extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}