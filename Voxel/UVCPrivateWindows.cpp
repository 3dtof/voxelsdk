/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCPrivateWindows.h"

#include "USBSystem.h"
#include "USBSystemPrivateWindows.h"

#include <dshow.h>
#include <comutil.h>
#include <cfgmgr32.h>
#include "Logger.h"
#include <uuids.h>

namespace Voxel
{

UVCPrivate::UVCPrivate(DevicePtr usb)
{
  if (usb->interfaceID() != Device::USB)
    return;

  USBSystem sys;
  String devicePath = sys.getDeviceNode((USBDevice &)*usb);

  if (!devicePath.size())
  {
    logger(LOG_ERROR) << "UVC: Could not get device path for device '" << usb->id() << "'" << std::endl;
    return;
  }

  DWORD devInst;

  if (!sys.getUSBSystemPrivate().getDevInst(devicePath, devInst))
  {
    logger(LOG_ERROR) << "UVC: Could not get devInst for device '" << usb->id() << "'" << std::endl;
    return;
  }

  HRESULT hr = S_OK;
  IMoniker *moniker;
  Ptr<ICreateDevEnum> devEnum;
  Ptr<IEnumMoniker> classEnum;

  void *p;

  CoInitializeEx(NULL, COINIT_MULTITHREADED);

  // Create the system device enumerator
  if ((hr = (CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC, IID_ICreateDevEnum, (void **)&p))) != S_OK)
  {
    logger(LOG_ERROR) << "UVC: Failed to get ICreateDevEnum enumerator" << std::endl;
    return;
  }

  devEnum = Ptr<ICreateDevEnum>((ICreateDevEnum *)p, [](ICreateDevEnum *i) { i->Release(); });

  // Create an enumerator for the video capture devices
  if ((hr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, (IEnumMoniker **)&p, 0)) != S_OK)
  {
    logger(LOG_ERROR) << "UVC: Failed to get VideoInputDeviceCategory class enumerator" << std::endl;
    return;
  }

  
  // If there are no enumerators for the requested type, then
  // CreateClassEnumerator will succeed, but pClassEnum will be NULL.
  if (p == NULL) 
  {
    logger(LOG_ERROR) << "UVC: Failed to get any video input devices in the enumerator" << std::endl;
    return;
  }

  classEnum = Ptr<IEnumMoniker>((IEnumMoniker *)p, [](IEnumMoniker *i) { i->Release(); });

  while ((hr = classEnum->Next(1, &moniker, NULL)) == S_OK)
  {
    IPropertyBag *propBag;
    if ((hr = moniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)&propBag)) != S_OK)
    {
      logger(LOG_WARNING) << "UVC: Could not get properties for current moniker" << std::endl;
      continue;
    }

    VARIANT varName;
    VariantInit(&varName);

    if ((hr = propBag->Read(L"DevicePath", &varName, 0)) != S_OK)
    {
      logger(LOG_WARNING) << "UVC: Could not get DevicePath for current moniker" << std::endl;
      continue;
    }

    _bstr_t b(varName.bstrVal);
    String devPath = b;
    VariantClear(&varName);

    DWORD dInst;

    DWORD parentDevInst;

    if (!sys.getUSBSystemPrivate().getDevInst(devPath, dInst, (LPGUID)&AM_KSCATEGORY_CAPTURE)) //AM_KSCATEGORY_VIDEO
    {
      logger(LOG_WARNING) << "UVC: Could not get DevInst for DevicePath = " << devPath << std::endl;
      continue;
    }
    
    if (CM_Get_Parent(&parentDevInst, dInst, 0) != CR_SUCCESS)
    {
      logger(LOG_WARNING) << "UVC: Could not get parent DevInst for DevicePath = " << devPath << std::endl;
      continue;
    }

    if (parentDevInst == devInst) // Check whether our USBDevice's DevInst matches with IMoniker's parent's DevInst matches
    {
      if (usb->channelID() >= 0)
      {
        auto x = devPath.find("&mi_");

        if (x != String::npos)
        {
          char *endptr;
          int channel = strtol(devPath.c_str() + x + 4, &endptr, 10);

          if (channel != usb->channelID())
            continue;
        }
        else
        {
          logger(LOG_ERROR) << "UVC: Could not find channel ID of current device '" << devPath << "'" << std::endl;
          continue;
        }
      }

      if ((hr = moniker->BindToObject(0, 0, IID_IBaseFilter, (void**)&p)) != S_OK)
      {
        logger(LOG_ERROR) << "UVC: Could not get the IBaseFilter for the current video input device" << std::endl;
        continue;
      }

      _captureFilter = Ptr<IBaseFilter>((IBaseFilter *)p, [](IBaseFilter *d) { d->Release(); });
      _captureFilter->AddRef();
      break;
    }
  }
}

UVCPrivate::~UVCPrivate()
{
  _captureFilter = nullptr;
  logger(LOG_DEBUG) << "UVCPrivate: cleanup" << std::endl;
  //CoUninitialize();
}

bool UVCPrivate::read(uint8_t *buffer, std::size_t size)
{
  return false;
}
  
}