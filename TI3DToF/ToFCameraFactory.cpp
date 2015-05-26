/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <ToFCameraFactory.h>
#include <Logger.h>

#include <CalculusCDKCamera.h>
#include <Voxel14Camera.h>
#include <HaddockCDKCamera.h>
#include <VoxelDCamera.h>
#include <TintinCDKCamera.h>

namespace Voxel
{
  
namespace TI
{
  

ToFCameraFactory::ToFCameraFactory(const String &name): DepthCameraFactory(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID2, "")),
    DevicePtr(new USBDevice(HADDOCK_CDK_VENDOR_ID, HADDOCK_CDK_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(HADDOCK_CDK_VENDOR_ID, CALCULUS_CDK_PRODUCT_ID, "")),
    DevicePtr(new USBDevice(VOXEL_D_VENDOR_ID, VOXEL_D_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(VOXEL_D_VENDOR_ID, VOXEL_D_PRODUCT_ID2, "")),
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_UVC, "")),
  });
}

DepthCameraPtr ToFCameraFactory::getDepthCamera(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == VOXEL_14_VENDOR_ID && (d.productID() == VOXEL_14_PRODUCT_ID1  || d.productID() == VOXEL_14_PRODUCT_ID2))
    {
      return DepthCameraPtr(new Voxel14Camera(device));
    }
    else if(d.vendorID() == HADDOCK_CDK_VENDOR_ID && (d.productID() == HADDOCK_CDK_PRODUCT_ID1))
    {
      return DepthCameraPtr(new HaddockCDKCamera(device));
    }
    else if(d.vendorID() == CALCULUS_CDK_VENDOR_ID && (d.productID() == CALCULUS_CDK_PRODUCT_ID))
    {
      return DepthCameraPtr(new CalculusCDKCamera(device));
    }
    else if(d.vendorID() == VOXEL_D_VENDOR_ID && (d.productID() == VOXEL_D_PRODUCT_ID1  || d.productID() == VOXEL_D_PRODUCT_ID2))
    {
      return DepthCameraPtr(new VoxelDCamera(device));
    }
    else if(d.vendorID() == TINTIN_CDK_VENDOR_ID && (d.productID() == TINTIN_CDK_PRODUCT_ID1  || d.productID() == TINTIN_CDK_PRODUCT_UVC))
    {
      return DepthCameraPtr(new TintinCDKCamera(device));
    }
  }
  
  return 0;
}

bool ToFCameraFactory::getChannels(Device &device, Vector<int> &channels)
{
  channels.resize(1);
  channels[0] = 0; // This supports only one channel for all supported devices
  return true;
}

Vector<GeneratorIDType> ToFCameraFactory::getSupportedGeneratorTypes()
{
  Vector<GeneratorIDType> d(2);
  d[0] = ((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_DEPTH_FRAME;
  d[1] = ((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_RAW_FRAME_PROCESSED;
  return d;
}

bool ToFCameraFactory::getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator)
{
  if(generatorID == (((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_DEPTH_FRAME))
  {
    if(frameType != DepthCamera::FRAME_DEPTH_FRAME)
      return false;
      
    frameGenerator = FrameGeneratorPtr(new ToFDepthFrameGenerator());
    return true;
  }
  else if(generatorID == (((TI_VENDOR_ID) << 16) | DepthCamera::FRAME_RAW_FRAME_PROCESSED))
  {
    if(frameType != DepthCamera::FRAME_RAW_FRAME_PROCESSED)
      return false;
    
    frameGenerator = FrameGeneratorPtr(new ToFFrameGenerator());
    return true;
  }
  else
  {
    logger(LOG_ERROR) << "ToFCameraFactory: Unknown generator ID = " << generatorID << std::endl;
    return false;
  }
}



extern "C" void TI3DTOF_EXPORT getDepthCameraFactory(DepthCameraFactoryPtr &ptr)
{
  ptr = DepthCameraFactoryPtr(new ToFCameraFactory("ti3dtof"));
}
 
extern "C" int TI3DTOF_EXPORT getABIVersion()
{
  return VOXEL_ABI_VERSION; // Return the Voxel ABI version which was used to compile this DepthCameraFactory
}
  
}
}