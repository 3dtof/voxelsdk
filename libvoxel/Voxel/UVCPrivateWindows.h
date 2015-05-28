/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_UVCPRIVATE_WINDOWS_H
#define VOXEL_UVCPRIVATE_WINDOWS_H

#include <Device.h>

#include <Windows.h>
#include <strmif.h>

namespace Voxel
{
  
/**
 * \addtogroup IO
 * @{
 */

  
class VOXEL_NO_EXPORT UVCPrivate
{
  Ptr<IBaseFilter> _captureFilter;
public:
  UVCPrivate(DevicePtr usb);
  
  bool isInitialized() { return (bool)_captureFilter; }


  IBaseFilter *getCaptureFilter() { return _captureFilter.get(); }
  
  bool read(uint8_t *buffer, std::size_t size);
  
  virtual ~UVCPrivate();
};
/**
 * @}
 */

}

#endif // UVCPRIVATE_H
