/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <ToFCameraFactory.h>
#include <Logger.h>

#include <Voxel14Camera.h>

namespace Voxel
{
  
namespace TI
{
  

ToFCameraFactory::ToFCameraFactory(const String &name): DepthCameraFactory(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID2, "")),
  });
}

DepthCameraPtr ToFCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interface() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_14_VENDOR_ID and (d.productID() == VOXEL_14_PRODUCT_ID1  or d.productID() == VOXEL_14_PRODUCT_ID2))
    {
      return DepthCameraPtr(new Voxel14Camera(device));
    }
  }
  
  return 0;
}

extern "C" void getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new ToFCameraFactory("ti3dtof"));
}
  
  
}
}