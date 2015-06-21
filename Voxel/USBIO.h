/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_USBIO_H
#define VOXEL_USBIO_H

#include "Device.h"
#include "USBSystem.h"
#include "Logger.h"

namespace Voxel
{
  
class VOXEL_EXPORT USBIO
{
protected:
  class USBIOPrivate;
  Ptr<USBIOPrivate> _usbIOPrivate;
  
public:
  USBIO(DevicePtr device);
  
  enum Direction
  {
    TO_DEVICE = 0,
    FROM_DEVICE = 1
  };
  
  enum RequestType
  {
    REQUEST_STANDARD = 0,
    REQUEST_CLASS = 1,
    REQUEST_VENDOR = 2,
    REQUEST_RESERVED = 3,
  };
  
  enum RecipientType
  {
    RECIPIENT_DEVICE = 0x00,
    RECIPIENT_INTERFACE = 0x01,
    RECIPIENT_ENDPOINT = 0x02,
    RECIPIENT_OTHER = 0x03,
  };
  
  bool controlTransfer(Direction direction, RequestType requestType, RecipientType recipientType, uint8_t request, uint16_t value, uint16_t index, 
                       uint8_t *data, uint16_t &length, bool needFullLength = true, long timeout = 1000);
  
  bool bulkTransfer(uint8_t endpoint, uint8_t *data, long toTransferLength, long &transferredLength, long timeout = 1000);
  
  bool resetBulkEndPoint(uint8_t endpoint);
  
  USBSystem &getUSBSystem();
  
  bool isInitialized();
  
  virtual ~USBIO() {}
};

typedef Ptr<USBIO> USBIOPtr;

}

#endif