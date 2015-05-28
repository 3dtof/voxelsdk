/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "TintinEEPROMDownloader.h"
#include "VoxelXUProgrammer.h"
#include "TintinCDKCamera.h"

namespace Voxel
{
  
namespace TI
{
  
uint8_t bitReverseTable[256] = {
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
  
  
bool TintinEEPROMDownloader::_configureForDownload()
{
  // Perform reset
  USBDevice &d = (USBDevice &)*_device;
  DevicePtr controlDevice;

  if (d.productID() == TINTIN_CDK_PRODUCT_UVC)
  {
    logger(LOG_INFO) << "TintinEEPROMDownloader: Reseting device..." << std::endl;
    _outStream << "Reseting device..." << std::endl;
    VoxelXUProgrammer p({}, _device);
    p.reset();
  
    // Wait for 10 seconds
    logger(LOG_INFO) << "TintinEEPROMDownloader: Waiting for device reenumeration..." << std::endl;
    _outStream << "Waiting for device reenumeration..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    controlDevice = DevicePtr(new USBDevice(TINTIN_CDK_VENDOR_ID, TINTIN_CDK_PRODUCT_ID1, _device->serialNumber(), 
                                            _device->channelID(), _device->description(), _device->serialIndex(), true));
  } else {
    controlDevice = _device;
  }
  
  _setProgress(10);
  
  _usbIO.reset(); // Remove reference to USBIO structure
  _usbIO = USBIOPtr(new USBIO(controlDevice));
  
  if(!_usbIO->isInitialized())
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Could not open device" << std::endl;
    _outStream << "Could not open device" << std::endl;
    return false;
  }
  
  // Write init
  uint8_t writeInit[] = { 0x06 };
  if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      0x17, 0x00, 0x00, writeInit, 1))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to init write in EEPROM" << std::endl;
    _outStream << "Failed to init write in EEPROM" << std::endl;
    return false;
  }

  // JEDEC ID
  uint8_t jedecID[3];
  if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      0x1D, 0x00, 0x00, jedecID, 3))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to read JEDEC ID" << std::endl;
    _outStream << "Failed to read JEDEC ID" << std::endl;
    return false;
  }
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: EEPROM JEDEC ID = " << std::hex << "0x" << (uint)jedecID[0] << (uint)jedecID[1] << (uint)jedecID[2] << std::endl;
  _outStream << "EEPROM JEDEC ID = " << std::hex << "0x" << (uint)jedecID[0] << (uint)jedecID[1] << (uint)jedecID[2] << std::endl;;
  
  uint8_t eepromStatus;
  if(!_getEEPROMStatus(eepromStatus))
    return false;
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  _outStream << "EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  
  if(!_printEEPROMFirst64Bytes())
    return false;
  
  // Write enable
  if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
    0x1E, 0x00, 0x00, writeInit, 1))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to enable write" << std::endl;
    _outStream << "Failed to enable write" << std::endl;
    return false;
  }
  
  if(!_getEEPROMStatus(eepromStatus))
    return false;
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: Write enable, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  _outStream << "Write enable, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  
  
  // EEPROM Erase
  uint8_t eepromEraseCommand[] = {0xC7};
  if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
    0x1E, 0x00, 0x00, eepromEraseCommand, 1))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to erase EEPROM" << std::endl;
    _outStream << "Failed to erase EEPROM" << std::endl;
    return false;
  }
  
  if(!_getEEPROMStatus(eepromStatus))
    return false;
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: Erase, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  _outStream << "Erase, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  
  _setProgress(15);
  
  // Wait for 30 seconds
  logger(LOG_INFO) << "TintinEEPROMDownloader: Waiting for erase..." << std::endl;
  _outStream << "Waiting for erase..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(30));
  
  if(!_getEEPROMStatus(eepromStatus))
    return false;
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: Erase done, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  _outStream << "Erase done, EEPROM Status = " << std::hex << "0x" << (uint)eepromStatus << std::endl;
  
  if(!_printEEPROMFirst64Bytes())
    return false;
  
  _setProgress(40);
  
  return true;
}

bool TintinEEPROMDownloader::_getEEPROMStatus(uint8_t &eepromStatus)
{
  // EEPROM status
  uint8_t eepromStat[1];
  if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
    0x1B, 0x00, 0x00, eepromStat, 1))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to read EEPROM status" << std::endl;
    _outStream << "Failed to read EEPROM status" << std::endl;
    return false;
  }
  
  eepromStatus = eepromStat[0];
  return true;
}


bool TintinEEPROMDownloader::_printEEPROMFirst64Bytes()
{
  uint8_t bytes[64];
  
  if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
    0x19, 0x00, 0x00, bytes, 64))
  {
    logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to read EEPROM bytes" << std::endl;
    _outStream << "Failed to read EEPROM bytes" << std::endl;
    return false;
  }
  
  logger(LOG_INFO) << "First 64 bytes = ";
  _outStream << "First 64 bytes = ";
  for(auto i = 0; i < 64; i++)
  {
    logger << "0x" << std::hex << (uint)bytes[i] << " ";
    _outStream << "0x" << std::hex << (uint)bytes[i] << " ";
  }
  logger << std::endl;
  _outStream << std::endl;
  return true;
}



bool TintinEEPROMDownloader::_download(InputFileStream &file, long unsigned int filesize)
{
  logger(LOG_INFO) << "TintinEEPROMDownloader: Starting EEPROM write..." << std::endl;
 _outStream << "Starting EEPROM write..." << std::endl;
  
  Vector<uint8_t> data(filesize), dataReverse;
  
  dataReverse.reserve(filesize);
  
  file.read((char *)data.data(), filesize);
  
  // bit-reverse all the bytes
  for(auto d: data)
    dataReverse.push_back(bitReverseTable[d]);
  
  
  uint32_t stepSize = 64, startAddress = 0, bytesRemaining = filesize, i = 0, bytesToSend;
  
  float p = getProgress();
  
  while(bytesRemaining)
  {
    if(bytesRemaining > stepSize)
      bytesToSend = stepSize;
    else
      bytesToSend = bytesRemaining;
    
    
    // Write enable
    uint8_t writeInit[] = { 0x06 };
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      0x1E, 0x00, 0x00, writeInit, 1))
    {
      logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to enable write" << std::endl;
      _outStream << "Failed to enable write" << std::endl;
      return false;
    }
    
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      0x18, startAddress & 0xFFFF, (startAddress >> 16) & 0x00FF, dataReverse.data() + startAddress, bytesToSend))
    {
      logger(LOG_ERROR) << "TintinEEPROMDownloader: Failed to write bytes at address 0x" << std::hex << startAddress << std::endl;
      _outStream << "Failed to write bytes at address 0x" << std::hex << startAddress << std::endl;
      return false;
    }
    
    if(i % 8192 == 0)
    {
      logger(LOG_INFO) << "TintinEEPROMDownloader: Pending " << std::dec << bytesRemaining << " bytes..." << std::endl;
      _outStream << "Pending " << std::dec << bytesRemaining << " bytes..." << std::endl;
    }
    
    bytesRemaining -= bytesToSend;
    startAddress += bytesToSend;
    i++;
    
    _setProgress(p + (100 - p)*(startAddress*1.0/filesize));
  }
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: EEPROM writes finished..." << std::endl;
  _outStream << "EEPROM writes finished..." << std::endl;

  if(!_printEEPROMFirst64Bytes())
    return false;
  
  logger(LOG_INFO) << "TintinEEPROMDownloader: Done!" << std::dec << std::endl;
  _outStream << "Done!" << std::dec << std::endl;

  return true;
}

  
}
}
  