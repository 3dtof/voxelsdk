/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2016 Texas Instruments Inc.
 */

#include "HardwareSerializer.h"
#include "Timer.h"

//defines for SPCA reads/writes
#define EEPROM_START_ADDRESS 0x00b000 //start address for EEPROM data
#define EEPROM_SIZE 0x5000 // 20 kB of available total size for the current EEPROM
#define ADDRESS_CODE 7 //set UVC address for read/write
#define READ_WRITE_CODE 8 //read/write data at the set address
#define ERASE_CODE 9 //erase data before writing
#define ADDRESS_SIZE 3 //3 bytes is the address size where calibration data is written
#define DATA_SIZE 32 //Size of data to be read/written

namespace Voxel
{
  
bool HardwareSerializer::getSize(uint32_t &size)
{ 
  if(!_usbIO && !_xu)
  {
    logger(LOG_ERROR) << "HardwareSerializer: USBIO not initialized." << std::endl;
    return false;
  }

  if(_usbIO)
  {
    uint8_t data[4];
    uint16_t length = 4;
    if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      _sizeRequestCode, 0, 0, data, length))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to read available size in hardware." << std::endl;
      return false;
    }
    
    size = *(uint32_t *)&data[0];
    
    return true;
  }

  if(_xu)
  {
    if(!_initXUReadWrite(EEPROM_START_ADDRESS)) // Just checking if the EEPROM is working. Size of the EEPROM is stored in Software
    {
      logger(LOG_ERROR) << "HardwareSerializer: Cannot inastantiate read from XU" << std::endl;
      return false;
    }

    size = EEPROM_SIZE;
    return true;
  }
}

bool HardwareSerializer::read(Version &version, TimeStampType &knownTimestamp, SerializedObject &so)
{
  if(!_usbIO && !_xu)
  {
    logger(LOG_ERROR) << "HardwareSerializer: IO not initialized." << std::endl;
    return false;
  }

  if(_usbIO)
  
  {
    Timer t;
    TimeStampType d = t.getCurrentRealTime();
    
    uint8_t data[9 + 2 + sizeof(TimeStampType)];
    
    uint16_t l = sizeof(data);
    
    if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, _ioRequestCode, 0, 0, data, l))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to read config size in hardware." << std::endl;
      return false;
    }
    
    if(data[0] != 'V' || data[1] != 'O' || data[2] != 'X' || data[3] != 'E' || data[4] != 'L')
    {
      logger(LOG_ERROR) << "HardwareSerializer: Invalid config data in hardware." << std::endl;
      return false;
    }
    
    // Ignoring the version info for now
    
    //std::cout << "Timestamp in hardware = " << *(TimeStampType *)&data[7] << ", expected = " << knownTimestamp << std::endl;
    
    if(*(TimeStampType *)&data[7] == knownTimestamp)
    {
      so.resize(0);
      return true;
    }
    
    int32_t length = *(uint32_t *)&data[7 + sizeof(TimeStampType)];
    
    uint32_t totalSize;
    
    if(!getSize(totalSize))
      return false;
    
    if(length > totalSize)
    {
      logger(LOG_ERROR) << "HardwareSerializer: Length of config data is greater than available size in hardware. (" 
      << length << " > " << totalSize << ")" << std::endl;
      return false;
    }
    
    so.resize(length);
    
    uint32_t offset = l, localOffset = 0;
    
    const uint32_t maxTransferLength = 4096;
    
    while(length > 0)
    {
      l = length;
      
      if(l > maxTransferLength) l = maxTransferLength;
      
      //std::cout << "l = " << l << std::endl;
      
      uint16_t offsetLower = (offset & 0xFFFF);
      uint16_t offsetUpper = (offset >> 16);
      
      if(!_usbIO->controlTransfer(USBIO::FROM_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
        _ioRequestCode, offsetUpper, offsetLower, (uint8_t *)so.getBytes().data() + localOffset, l) || l == 0)
      {
        logger(LOG_ERROR) << "HardwareSerializer: Failed to read config data from hardware." << std::endl;
        return false;
      }
      
      length -= l;
      offset += l;
      localOffset += l;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Received " << so.size() << " bytes from hardware in " << (t.getCurrentRealTime() - d)*1E-6 << " s" << std::endl;
    
    return true;
  }

  if(_xu)
  { 
    Timer t;
    TimeStampType d = t.getCurrentRealTime();
    
    uint8_t data[32];
    
    uint16_t l = sizeof(data); //keeping this 32 bytes, so that there isn't  a problem with read/write
    
    if(!_initXUReadWrite(EEPROM_START_ADDRESS) || !(_xu->getControl(READ_WRITE_CODE, sizeof(data), data)))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to read config size in hardware." << std::endl;
      return false;
    }
    
    if(data[0] != 'V' || data[1] != 'O' || data[2] != 'X' || data[3] != 'E' || data[4] != 'L')
    {
      logger(LOG_ERROR) << "HardwareSerializer: Invalid config data in hardware." << std::endl;
      logger(LOG_ERROR) << data[0] << data[1] << data[2] << data[3] << data[4] << std::endl;
      return false;
    }
    
    // Ignoring the version info for now
    
    std::cout << "Timestamp in hardware = " << *(TimeStampType *)&data[7] << ", expected = " << knownTimestamp << std::endl;
    
    if(*(TimeStampType *)&data[7] == knownTimestamp)
    {
      so.resize(0);
      return true;
    }
    
    int32_t length = *(uint32_t *)&data[7 + sizeof(TimeStampType)];
    
    uint32_t totalSize;
    
    if(!getSize(totalSize))

      { logger(LOG_ERROR) << "Cannot get total size" << std::endl;
        return false;
      }
    
    if(length > totalSize)
    {
      logger(LOG_ERROR) << "HardwareSerializer: Length of config data is greater than available size in hardware. (" 
      << length << " > " << totalSize << ")" << std::endl;
      return false;
    }
    
    so.resize(length);
    
    uint32_t offset = l, localOffset = 0;
    
    
    while(length > 0)
    {
      l = length;
      
      if(l > DATA_SIZE) l = DATA_SIZE;
      
      if(!_initXUReadWrite(EEPROM_START_ADDRESS + offset) || !(_xu->getControl(READ_WRITE_CODE, DATA_SIZE, (uint8_t *)so.getBytes().data() + localOffset)) || l == 0)
      {
        logger(LOG_ERROR) << "HardwareSerializer: Failed to read config data from hardware." << std::endl;
        return false;
      }
      
      length -= l;
      offset += l;
      localOffset += l;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Received " << so.size() << " bytes from hardware in " << (t.getCurrentRealTime() - d)*1E-6 << " s" << std::endl;
    
    return true;
    }
}


bool HardwareSerializer::write(Version &version, TimeStampType &timestamp, SerializedObject &so)
{
  if(!_usbIO && !_xu)
  {
    logger(LOG_ERROR) << "HardwareSerializer: USBIO not initialized." << std::endl;
    return false;
  }
  
  if(_usbIO)
  {
    Timer t;
    TimeStampType d = t.getCurrentRealTime();
    
    int32_t length = so.size();
    uint32_t totalSize;
    
    if(!getSize(totalSize))
      return false;
    
    if(length + 9 > totalSize)
    {
      logger(LOG_ERROR) << "HardwareSerializer: Length of config data is greater than available size in hardware. (" 
      << length + 9 << " > " << totalSize << ")" << std::endl;
      return false;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Length of data to hardware = " << length << std::endl;
    
    uint8_t data[9 + 2 + sizeof(TimeStampType)];
    
    data[0] = 'V';
    data[1] = 'O';
    data[2] = 'X';
    data[3] = 'E';
    data[4] = 'L';
    
    data[5] = version.major;
    data[6] = version.minor;
    
    *(TimeStampType *)&data[7] = timestamp;
    
    *(uint32_t *)&data[7 + sizeof(TimeStampType)] = so.size();
    
    uint16_t l = sizeof(data);
    if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
      _ioRequestCode, 0, 0, data, l))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to read config size in hardware." << std::endl;
      return false;
    }
    
    uint32_t offset = l, localOffset = 0;
    
    while(length > 0)
    {
      logger(LOG_DEBUG) << "HardwareSerializer: Transferred till offset = " << offset << std::endl;
      
      l = 64 - (offset % 64);
      
      if(l > length) l = length;
      
      uint16_t offsetLower = (offset & 0xFFFF);
      uint16_t offsetUpper = (offset >> 16);
      
      if(!_usbIO->controlTransfer(USBIO::TO_DEVICE, USBIO::REQUEST_VENDOR, USBIO::RECIPIENT_DEVICE, 
        _ioRequestCode, offsetUpper, offsetLower, (uint8_t *)so.getBytes().data() + localOffset, l, false, 1000))
      {
        logger(LOG_ERROR) << "HardwareSerializer: Failed to read config data from hardware." << std::endl;
        return false;
      }
      
      length -= l;
      offset += l;
      localOffset += l;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Transferred " << so.size() << " bytes to hardware in " << (t.getCurrentRealTime() - d)*1E-6 << " s" << std::endl;
    
    return true;
  }

  if(_xu)
  {
    Timer t;
    TimeStampType d = t.getCurrentRealTime();
    
    int32_t length = so.size();
    uint32_t totalSize;
    
    if(!getSize(totalSize))
      return false;
    
    if(length + 9 > totalSize)
    {
      logger(LOG_ERROR) << "HardwareSerializer: Length of config data is greater than available size in hardware. (" 
      << length + 9 << " > " << totalSize << ")" << std::endl;
      return false;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Length of data to hardware = " << length << std::endl;
    
    uint8_t data[32];
    
    data[0] = 'V';
    data[1] = 'O';
    data[2] = 'X';
    data[3] = 'E';
    data[4] = 'L';
    
    data[5] = version.major;
    data[6] = version.minor;
    
    *(TimeStampType *)&data[7] = timestamp;
    
    *(uint32_t *)&data[7 + sizeof(TimeStampType)] = so.size();
    
    uint16_t l = sizeof(data);

    uint8_t eraseData[3];
    eraseData[0] = EEPROM_START_ADDRESS & 0xFF;
    eraseData[1] = (EEPROM_START_ADDRESS >> 8) & 0xFF;
    eraseData[2] = (EEPROM_START_ADDRESS >> 16) & 0xFF;

    if(!_xu->setControl(ERASE_CODE, sizeof(eraseData), eraseData)||!_initXUReadWrite(EEPROM_START_ADDRESS) || !_xu->setControl(READ_WRITE_CODE, DATA_SIZE, data))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to read config size in hardware." << std::endl;
      return false;
    }
    
    uint32_t offset = l, localOffset = 0;
    
    while(length > 0)
    {
      logger(LOG_DEBUG) << "HardwareSerializer: Transferred till offset = " << offset << std::endl;
      
      if(!_initXUReadWrite(EEPROM_START_ADDRESS + offset) || !_xu->setControl(READ_WRITE_CODE, DATA_SIZE, (uint8_t *)so.getBytes().data() + localOffset))
      {
        logger(LOG_ERROR) << "HardwareSerializer: Failed to write config data to hardware." << std::endl;
        return false;
      }

      // if(localOffset%128 == 0 && localOffset%256 !=0) // writing again, single write not working
      //     {
      //       uint32_t address = EEPROM_START_ADDRESS + offset; 
      //       eraseData[0] = address & 0xFF;
      //       eraseData[1] = (address >> 8) & 0xFF;
      //       eraseData[2] = (address >> 16) & 0xFF;
      //       if(!_xu->setControl(ERASE_CODE, sizeof(eraseData), eraseData))
      //         return false;
      //     }
      
      length -= l;
      offset += l;
      localOffset += l;
    }
    
    logger(LOG_INFO) << "HardwareSerializer: Transferred " << so.size() << " bytes to hardware in " << (t.getCurrentRealTime() - d)*1E-6 << " s" << std::endl;
    
    return true;
  }
}

bool HardwareSerializer::writeToFile(const String &filename, Version &version, TimeStampType &timestamp, SerializedObject &so)
{
  OutputFileStream fs(filename, std::ios::out | std::ios::binary);
  
  if(!fs.good())
  {
    logger(LOG_ERROR) << "HardwareSerializer: Could not open file '" << filename << "'" << std::endl;
    return false;
  }
  
  fs.write((const char *)&version, sizeof(version));
  fs.write((const char *)&timestamp, sizeof(timestamp));
  fs.write((const char *)so.getBytes().data(), so.size());
  
  fs.close();
  return true;
}

bool HardwareSerializer::_initXUReadWrite(uint32_t address)
{
    uint8_t data1[3];
    data1[0] = address & 0xFF;
    data1[1] = (address >> 8) & 0xFF;
    data1[2] = (address >> 16) & 0xFF;

    if(!_xu->setControl(ADDRESS_CODE, ADDRESS_SIZE, data1))
    {
      logger(LOG_ERROR) << "HardwareSerializer: Failed to instantate read/write." << std::endl;
      return false;
    }
}
  
}