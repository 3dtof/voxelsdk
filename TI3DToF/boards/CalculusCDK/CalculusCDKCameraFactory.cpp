/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CalculusCDKCameraFactory.h"
#include <Logger.h>

#include "CalculusCDKCamera.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{
  

CalculusCDKCameraFactory::CalculusCDKCameraFactory(const String &name): ToFCameraFactoryBase(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(CALCULUS_CDK_VENDOR_ID, CALCULUS_CDK_PRODUCT_ID, "")),
  });
}

DepthCameraPtr CalculusCDKCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == CALCULUS_CDK_VENDOR_ID && (d.productID() == CALCULUS_CDK_PRODUCT_ID))
    {
      return DepthCameraPtr(new CalculusCDKCamera(device));
    }
  }
  
  return 0;
}


extern "C" void SYMBOL_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new CalculusCDKCameraFactory("CalculusCDK"));
}
 
extern "C" int SYMBOL_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}