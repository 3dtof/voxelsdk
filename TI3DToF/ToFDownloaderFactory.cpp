/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFDownloaderFactory.h"
#include "TintinCDKCamera.h"
#include "TintinEEPROMDownloader.h"

namespace Voxel
{
  
namespace TI
{

ToFDownloaderFactory::ToFDownloaderFactory(const String &name): DownloaderFactory(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_ID1, "")),
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_UVC, "")),
  });
}


DownloaderPtr ToFDownloaderFactory::getDownloader(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == TINTIN_CDK_VENDOR_ID && (d.productID() == TINTIN_CDK_PRODUCT_ID1  || d.productID() == TINTIN_CDK_PRODUCT_UVC))
    {
      return DownloaderPtr(new TintinEEPROMDownloader(device));
    }
  }
  
  return 0;
}

extern "C" void TI3DTOF_EXPORT getDownloaderFactory(DownloaderFactoryPtr &ptr)
{
  ptr = DownloaderFactoryPtr(new ToFDownloaderFactory("ti3dtof"));
}

  
}
}