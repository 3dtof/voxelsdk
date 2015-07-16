/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "CameraSystem.h"

#include "SimpleOpt.h"
#include "Common.h"
#include "Logger.h"
#include "UVCStreamer.h"
#include <iomanip>
#include <fstream>

using namespace Voxel;

enum Options
{
  VENDOR_ID = 0,
  PRODUCT_ID = 1,
  SERIAL_NUMBER = 2,
  CREATE_PROFILE = 3,
  PARENT_PROFILE_ID = 4,
  PROFILE_ID = 5,
  SECTION = 6,
  PARAM_NAME = 7,
  PARAM_VALUE = 8,
  LIST_PROFILES = 9,
  REMOVE_PROFILE = 10,
  WRITE_TO_EEPROM = 11
};

Vector<CSimpleOpt::SOption> argumentSpecifications = 
{
  { VENDOR_ID,        "-v", SO_REQ_SEP, "Vendor ID of the USB device (hexadecimal)"}, // Only worker count is needed here
  { PRODUCT_ID,       "-p", SO_REQ_SEP, "Comma separated list of Product IDs of the USB devices (hexadecimal)"},
  { SERIAL_NUMBER,    "-s", SO_REQ_SEP, "Serial number of the USB device (string)"},
  { CREATE_PROFILE,   "-c", SO_REQ_SEP, "Should create a new profile? If given, then name is to specified"},
  { PARENT_PROFILE_ID,"-d", SO_REQ_SEP, "Parent profile ID used while creating a new profile [default = -1]."},
  { PROFILE_ID,       "-i", SO_REQ_SEP, "ID of the camera profile"},
  { SECTION,          "-t", SO_REQ_SEP, "Name of the configuration section"},
  { PARAM_NAME,       "-n", SO_REQ_SEP, "Name of the parameter to read/write"},
  { PARAM_VALUE,      "-u", SO_REQ_SEP, "Value for the parameter. This will be written. If not given, then the parameter will be read."},
  { LIST_PROFILES,    "-l", SO_NONE,    "List all profiles for this camera"},
  { REMOVE_PROFILE,   "-r", SO_NONE,    "Remove profile selected with -i option"},
  { WRITE_TO_EEPROM,  "-w", SO_NONE,    "Write to EEPROM, the profile selected with -i option"},
  SO_END_OF_OPTIONS
};

void help()
{
  std::cout << "CameraSystemConfigTest v1.0" << std::endl;
  
  CSimpleOpt::SOption *option = argumentSpecifications.data();
  
  while(option->nId >= 0)
  {
    std::cout << option->pszArg << " " << option->helpInfo << std::endl;
    option++;
  }
}


int main(int argc, char *argv[])
{
  CSimpleOpt s(argc, argv, argumentSpecifications);
  
  logger.setDefaultLogLevel(LOG_ERROR);
  
  uint16_t vid = 0;
  
  Vector<uint16_t> pids;
  String serialNumber;
  
  bool listProfiles = false;
  
  bool createProfile = false, removeProfile = false, writeToEEPROM = false;
  String profileName;
  int profileID = -1, parentProfileID = -1;
  
  bool readParam = true;
  String section, paramName, paramValue;
  
  char *endptr;
  Vector<String> splits;
  
  
  while (s.Next())
  {
    if (s.LastError() != SO_SUCCESS)
    {
      std::cout << s.GetLastErrorText(s.LastError()) << ": '" << s.OptionText() << "' (use -h to get command line help)" << std::endl;
      help();
      return -1;
    }
    
    //std::cout << s.OptionId() << ": " << s.OptionArg() << std::endl;
    
    Vector<String> splits;
    switch (s.OptionId())
    {
      case VENDOR_ID:
        vid = (uint16_t)strtol(s.OptionArg(), &endptr, 16);
        break;
        
      case PRODUCT_ID:
        split(s.OptionArg(), ',', splits);
        
        for(auto &s1: splits)
          pids.push_back((uint16_t)strtol(s1.c_str(), &endptr, 16));
        
        break;
        
      case SERIAL_NUMBER:
        serialNumber = s.OptionArg();
        break;
      
      case CREATE_PROFILE:
        createProfile = true;
        profileName = s.OptionArg();
        break;
        
      case PARENT_PROFILE_ID:
        parentProfileID = atoi(s.OptionArg());
        break;
        
      case PROFILE_ID:
        profileID = atoi(s.OptionArg());
        break;
        
      case SECTION:
        section = s.OptionArg();
        break;
        
      case PARAM_NAME:
        paramName = s.OptionArg();
        break;
        
      case PARAM_VALUE:
        readParam = false;
        paramValue = s.OptionArg();
        break;
        
      case LIST_PROFILES:
        listProfiles = true;
        break;
        
      case REMOVE_PROFILE:
        removeProfile = true;
        break;
        
      case WRITE_TO_EEPROM:
        writeToEEPROM = true;
        break;
        
      default:
        help();
        break;
    };
  }
  
  if(vid == 0 || pids.size() == 0 || pids[0] == 0)
  {
    logger(LOG_ERROR) << "Required argument missing." << std::endl;
    help();
    return -1;
  }
  
  CameraSystem sys;
  
  // Get all valid detected devices
  const Vector<DevicePtr> &devices = sys.scan();
  
  DevicePtr toConnect;
  
  std::cout << "Detected devices: " << std::endl;
  for(auto &d: devices)
  {
    std::cout << d->id() << std::endl;
    
    if(d->interfaceID() == Device::USB)
    {
      USBDevice &usb = (USBDevice &)*d;
      
      if(usb.vendorID() == vid && (serialNumber.size() == 0 || usb.serialNumber() == serialNumber))
      {
        for(auto pid: pids)
          if(usb.productID() == pid)
            toConnect = d;
      }
    }
  }
  
  if(!toConnect)
  {
    logger(LOG_ERROR) << "No valid device found for the specified VID:PID:serialnumber" << std::endl;
    return -1;
  }
  
  DepthCameraPtr depthCamera = sys.connect(toConnect);
  
  if(!depthCamera)
  {
    logger(LOG_ERROR) << "Could not load depth camera for device " << toConnect->id() << std::endl;
    return -1;
  }
  
  if(!depthCamera->isInitialized())
  {
    logger(LOG_ERROR) << "Depth camera not initialized for device " << toConnect->id() << std::endl;
    return -1;
  }
  
  std::cout << "Successfully loaded depth camera for device " << toConnect->id() << std::endl;
  
  if(listProfiles)
  {
    const Map<int, String> &profiles = depthCamera->getCameraProfileNames();
    
    for(auto &p: profiles)
    {
      std::cout << p.first << ", " << p.second;
      
      ConfigurationFile *c = depthCamera->configFile.getCameraProfile(p.first);
      
      if(c && c->getLocation() == ConfigurationFile::IN_CAMERA)
        std::cout << " (HW)";
      
      std::cout << std::endl;
    }
    
    return 0;
  }
  
  int id;
  
  if(createProfile)
  {
    id = depthCamera->addCameraProfile(profileName, parentProfileID);
    if(id < 0)
    {
      logger(LOG_ERROR) << "Failed to create new camera profile with name '" << profileName << "' and parent = " << parentProfileID << std::endl;
      return -1;
    }
    
    std::cout << "Created profile with id = " << id << std::endl;
  }
  else
  {
    id = profileID;
  }
  
  ConfigurationFile *configFile = depthCamera->configFile.getCameraProfile(id);
  
  if(!configFile)
  {
    logger(LOG_ERROR) << "Could not find config file for id = " << id << std::endl;
    return -1;
  }
  
  bool needToWrite = false;
  if(section.size() == 0 || paramName.size() == 0 || (!readParam && paramValue.size() == 0))
  {
    logger(LOG_INFO) << "One of the requirement parameter to read/write parameter is missing." << std::endl;
  }
  else
  {
    if(readParam)
    {
      if(configFile->isPresent(section, paramName))
        std::cout << "Param value for section = " << section << ", name = " << paramName 
          << ", value = " << configFile->get(section, paramName) << std::endl;
      else
        logger(LOG_ERROR) << "Param not found, section = " << section << ", name = " << paramName << std::endl;
    }
    else
    {
      if(!configFile->set(section, paramName, paramValue))
      {
        std::cout << "Failed to set parameter, section = " << section << ", name = " << paramName 
        << ", value = " << paramValue << std::endl;
      }
      else
      {
        std::cout << "Successfully set parameter, section = " << section << ", name = " << paramName 
        << ", value = " << paramValue << std::endl;
        needToWrite = true;
      }
    }
  }
  
  if(needToWrite)
  {
    if(configFile->getLocation() == ConfigurationFile::IN_CAMERA)
    {
      if(!depthCamera->configFile.writeToHardware())
      {
        logger(LOG_ERROR) << "Failed to save configuration for id = " << id << std::endl;
        return -1;
      }
    }
    else if(!configFile->write())
    {
      logger(LOG_ERROR) << "Failed to save configuration for id = " << id << std::endl;
      return -1;
    }
    return 0;
  }
  
  if(writeToEEPROM)
  {
    if(!depthCamera->saveCameraProfileToHardware(id))
    {
      logger(LOG_ERROR) << "Failed to save configuration for id = " << id << " to EEPROM." << std::endl;
      return -1;
    }
    
    std::cout << "Profile saved to hardware with id = " << id << std::endl;
    return 0;
  }
  
  if(removeProfile)
  {
    char ch;
    std::cout << "Are you sure you want to remove profile with id '" << id << "' [y/n]? ";
    std::cin >> ch;
    
    if(ch == 'y' || ch == 'Y')
    {
      if(!depthCamera->configFile.removeCameraProfile(id))
      {
        std::cout << "Failed to remove camera profile '" << id << "'" << std::endl;
        return -1;
      }
      else
        std::cout << "Successfully removed camera profile '" << id << "'" << std::endl;
    }
    return 0;
  }
  
  return 0;
}