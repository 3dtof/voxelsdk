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

DevicePtr ToFCameraFactory::_getControlDevice(DevicePtr device)
{
  if(device->interface() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_14_VENDOR_ID and (d.productID() == VOXEL_14_PRODUCT_ID1 or d.productID() == VOXEL_14_PRODUCT_ID2))
    {
      return DevicePtr(new USBDevice(d.vendorID(), VOXEL_14_PRODUCT_ID2, d.serialNumber()));
    }
  }
  
  return 0;
}

DevicePtr ToFCameraFactory::_getStreamDevice(DevicePtr device)
{
  return _getControlDevice(device); // same as control device
}

DepthCameraPtr ToFCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interface() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_14_VENDOR_ID and (d.productID() == VOXEL_14_PRODUCT_ID1  or d.productID() == VOXEL_14_PRODUCT_ID2))
    {
      DevicePtr control = _getControlDevice(device), stream = _getStreamDevice(device);
      
      if(!control or !stream)
      {
        log(ERROR) << "ToFCameraFactory: Failed to get control or stream device for " << device->id() << endl;
        return 0;
      }
      
      return DepthCameraPtr(new Voxel14Camera(device, control, stream));
    }
  }
  
  return 0;
}

extern "C" DepthCameraFactoryPtr getDepthCameraFactory()
{
  return DepthCameraFactoryPtr(new ToFCameraFactory("ti3dtof"));
}
  
  
}
}