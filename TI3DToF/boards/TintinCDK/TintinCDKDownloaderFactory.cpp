/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "TintinCDKDownloaderFactory.h"
#include "TintinCDKCamera.h"
#include "TintinEEPROMDownloader.h"

#include "SymbolExports.h"

namespace Voxel
{
  
namespace TI
{

TintinCDKDownloaderFactory::TintinCDKDownloaderFactory(const String &name): DownloaderFactory(name)
{
  _addSupportedDevices({
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_BULK, "")),
    DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_UVC, "")),
  });
}


DownloaderPtr TintinCDKDownloaderFactory::getDownloader(DevicePtr device)
{
  if(device->interfaceID() == Device::USB)
  {
    USBDevice &d = (USBDevice &)*device;
    
    if(d.vendorID() == TINTIN_CDK_VENDOR_ID && 
      (d.productID() == TINTIN_CDK_PRODUCT_BULK || d.productID() == TINTIN_CDK_PRODUCT_UVC))
    {
      return DownloaderPtr(new TintinEEPROMDownloader(device));
    }
  }
  
  return 0;
}

extern "C" void SYMBOL_EXPORT getDownloaderFactory(DownloaderFactoryPtr &ptr)
{
  ptr = DownloaderFactoryPtr(new TintinCDKDownloaderFactory("TintinCDK"));
}

  
}
}