
/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCXU.h"
#include "Logger.h"
#include "USBSystem.h"

#include <string.h>

#ifdef LINUX
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include "UVCPrivateLinux.h"
#elif defined(WINDOWS)
#include "UVCPrivateWindows.h"
#include <Windows.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>
#include <comdef.h>

DEFINE_GUIDSTRUCT("8A0F88DD-BA1C-5449-8A25-F7875967F0F7", PROPSETID_FX2_XU);
DEFINE_GUIDSTRUCT("0FB885C3-68C2-4547-90F7-8F47579D95FC", PROPSETID_SUNPLUS_XU);
#define PROPSETID_FX2_XU DEFINE_GUIDNAMED(PROPSETID_FX2_XU)
#define PROPSETID_SUNPLUS_XU DEFINE_GUIDNAMED(PROPSETID_SUNPLUS_XU)
#define STATIC_IID_IKsControl \
0x28F54685L, 0x06FD, 0x11D2, 0xB2, 0x7A, 0x00, 0xA0, 0xC9, 0x22, 0x31, 0x96
DEFINE_GUID(IID_IKsControl,
  0x28F54685L, 0x06FD, 0x11D2, 0xB2, 0x7A, 0x00, 0xA0, 0xC9, 0x22, 0x31, 0x96);

#endif

namespace Voxel
{

class UVCXU::UVCXUPrivate
{
public:
#ifdef WINDOWS
  Ptr<IKsControl> ksControl;
#endif
  UVCXUPrivate() {}
  virtual ~UVCXUPrivate() {}
};
  
UVCXU::UVCXU(DevicePtr usb, int xuID, uint8_t index): UVC(usb), _xuID(xuID), _uvcXUPrivate(Ptr<UVCXUPrivate>(new UVCXUPrivate())), _index(index)
{
#ifdef WINDOWS
  if (!UVC::isInitialized())
    return;

  HRESULT hr = S_OK;

  Ptr<IKsTopologyInfo> ksTopologyInfo;
  void *p;
  if ((hr = _uvcPrivate->getCaptureFilter()->QueryInterface(__uuidof(IKsTopologyInfo), &p)) != S_OK)
  {
    logger(LOG_ERROR) << "UVCXU: Could not get IksTopologyInfo structure associated with video input device '" << usb->id() << "'" << std::endl;
    return;
  }

  ksTopologyInfo = Ptr<IKsTopologyInfo>((IKsTopologyInfo *)p, [](IKsTopologyInfo *d) { d->Release(); });

  // Retrieve the number of nodes in the filter
  DWORD numNodes = 0;
  if ((hr = ksTopologyInfo->get_NumNodes(&numNodes)) != S_OK || numNodes == 0)
  {
    logger(LOG_ERROR) << "UVCXU: Could not get number of nodes in the given IKsTopologyInfo for video input device '" << usb->id() << "'" << std::endl;
    return;
  }
  

  // Find the extension unit node that corresponds to the given GUID
  GUID guidNodeType;
  KSP_NODE ExtensionProp;
  ULONG bytesReturned = 0;
  for (unsigned int i = 0; i < numNodes; i++)
  {
    ksTopologyInfo->get_NodeType(i, &guidNodeType);
    if (IsEqualGUID(guidNodeType, KSNODETYPE_DEV_SPECIFIC))
    {
      if ((hr = ksTopologyInfo->CreateNodeInstance(i, IID_IKsControl, (void **)&p)) == S_OK)
      {
    	if (_index == 0)
		  ExtensionProp.Property.Set = PROPSETID_FX2_XU;// FIXME: This seems to be specific to Voxel-14. Need to handle this separately
    	else if (_index ==1)
		  ExtensionProp.Property.Set = PROPSETID_SUNPLUS_XU;
        ExtensionProp.Property.Id = 0;
        ExtensionProp.Property.Flags = KSPROPERTY_TYPE_SETSUPPORT | KSPROPERTY_TYPE_TOPOLOGY;
        ExtensionProp.NodeId = i;
        ExtensionProp.Reserved = 0;

        if ((hr = ((IKsControl *)p)->KsProperty((PKSPROPERTY)&ExtensionProp, sizeof(ExtensionProp), NULL, 0, &bytesReturned)) != S_OK)
        {
          ((IKsControl *)p)->Release();
          continue;
        }

        _uvcXUPrivate->ksControl = Ptr<IKsControl>((IKsControl *)p, [](IKsControl *d) { d->Release(); }); // Removed d->Release(); from delete function, to avoid a crash
        _xuID = i;
        break;
      }
    }
  }

  if (!_uvcXUPrivate->ksControl)
  {
    logger(LOG_ERROR) << "UVCXU: Could not get IKsControl instance associated with video input device '" << usb->id() << "'" << std::endl;
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for the device to get ready
#endif
}

bool UVCXU::isInitialized()
{
  return UVC::isInitialized()
#ifdef WINDOWS
    && _uvcXUPrivate->ksControl
#endif
    ;
}

UVCXU::~UVCXU() {}

bool UVCXU::getControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;

#ifdef LINUX  
  struct uvc_xu_control_query uvc;
  
  memset(&uvc, 0, sizeof(uvc));
  
  logger(LOG_DEBUG) << "UVCXU: get control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << (uint)value[0] << std::endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_GET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (getUVCPrivate().xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    logger(LOG_ERROR) << "UVCXU: " << _usb->id() << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
#elif defined(WINDOWS)
  KSP_NODE ExtensionProp;
  ULONG bytesReturned = 0;

  HRESULT hr;
  
  if (_index == 0)
	ExtensionProp.Property.Set = PROPSETID_FX2_XU;// FIXME: This seems to be specific to Voxel-14. Need to handle this separately
  else if (_index ==1)
		ExtensionProp.Property.Set = PROPSETID_SUNPLUS_XU;
  ExtensionProp.Property.Id = controlnumber;
  ExtensionProp.Property.Flags = KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
  ExtensionProp.NodeId = _xuID;

  if ((hr = _uvcXUPrivate->ksControl->KsProperty((PKSPROPERTY)&ExtensionProp, sizeof(ExtensionProp), (PVOID)value, size, &bytesReturned)) != S_OK)
  {
    logger(LOG_ERROR) << "UVCXU: " << _usb->id() << " KsProperty query failed. Error: " << _com_error(hr).ErrorMessage() << std::endl;
    return false;
  }

  if (bytesReturned != size)
  {
    logger(LOG_ERROR) << "UVCXU: " << _usb->id() << " KsProperty query returned only " << bytesReturned << " bytes, but expected " << size << " bytes." << std::endl;
    return false;
  }
#endif
  return true;  
}

bool UVCXU::setControl(int controlnumber, int size, uint8_t *value)
{
  if(!isInitialized())
    return false;
 
#ifdef LINUX
  struct uvc_xu_control_query uvc;
  
  memset(&uvc, 0, sizeof(uvc));
  
  logger(LOG_DEBUG) << "UVCXU: set control " << controlnumber << ", size " << size << ", value[0] 0x" << std::hex << (uint)value[0] << std::endl;
  
  uvc.unit = _xuID;
  uvc.selector = controlnumber;
  uvc.query = UVC_SET_CUR;
  uvc.size = size;
  uvc.data = value;
  
  if (getUVCPrivate().xioctl(UVCIOC_CTRL_QUERY, &uvc) == -1) 
  {
    logger(LOG_ERROR) << "UVCXU: " << _usb->id() << " UVCIOC_CTRL_QUERY failed.\n";
    return false;
  }
#elif defined(WINDOWS)
  KSP_NODE ExtensionProp;
  ULONG bytesReturned = 0;

  if (_index == 0)
	ExtensionProp.Property.Set = PROPSETID_FX2_XU;// FIXME: This seems to be specific to Voxel-14. Need to handle this separately
  else if (_index ==1)
	ExtensionProp.Property.Set = PROPSETID_SUNPLUS_XU;
  ExtensionProp.Property.Id = controlnumber;
  ExtensionProp.Property.Flags = KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
  ExtensionProp.NodeId = _xuID;

  if (_uvcXUPrivate->ksControl->KsProperty((PKSPROPERTY)&ExtensionProp, sizeof(ExtensionProp), (PVOID)value, size, &bytesReturned) != S_OK)
  {
    logger(LOG_ERROR) << "UVCXU: " << _usb->id() << " KsProperty query failed." << std::endl;
    return false;
  }

#endif
  
  return true;
}
  
}
