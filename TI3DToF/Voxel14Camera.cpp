/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Voxel14Camera.h"
#include "VoxelXUProgrammer.h"
#include <Logger.h>
#include <UVCStreamer.h>

namespace Voxel
{
  
namespace TI
{
  
Voxel14Camera::Voxel14Camera(Voxel::DevicePtr device): ToFHaddockCamera("Voxel14Camera", device)
{
  _init();
}

bool Voxel14Camera::_init()
{
  USBDevice &d = (USBDevice &)*_device;
  
  DevicePtr controlDevice;
  
  if(d.productID() == VOXEL_14_PRODUCT_ID1)
  {
    _downloader = Ptr<Downloader>(new USBDownloader(_device));
    if(!_downloader->download("OPT9220_0v27.fw")) // TODO: This needs to come from a configuration
    {
      log(ERROR) << "Voxel14Camera: Firmware download failed" << endl;
      return false;
    }
    else 
      std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait for device to get ready after loading firmware
      
    controlDevice = DevicePtr(new USBDevice(VOXEL_14_VENDOR_ID, VOXEL_14_PRODUCT_ID2, _device->serialNumber()));
  }
  else
    controlDevice = _device;
  
  _programmer = Ptr<RegisterProgrammer>(new VoxelXUProgrammer(controlDevice));
  _streamer = Ptr<Streamer>(new UVCStreamer(controlDevice));
  
  if(!_programmer->isInitialized() || !_streamer->isInitialized())
    return false;
  
  if(!ToFHaddockCamera::_init())
    return false;
  
  return true;
}



  
}
}