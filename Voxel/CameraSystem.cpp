/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CameraSystem.h"
#include <Logger.h>

#include <fstream>

#define LIB_MATCH_STRING ".so"

namespace Voxel
{
  
CameraSystem::CameraSystem()
{
  _init();
}


void CameraSystem::_init()
{
  Vector<String> paths;
  
  paths.clear();
  paths.push_back("/usr/lib/voxel");
  
  char *p = getenv("VOXEL_LIB_PATH");
  
  if(p != 0)
  {
    String p1(p);
    
    Vector<String> splits;
    
    split(p1, ':', splits);
    
    paths.reserve(paths.size() + splits.size());
    paths.insert(paths.end(), splits.begin(), splits.end());
  }
  
  if(log.getDefaultLogLevel() >= DEBUG) 
  {
    for(auto i = 0; i < paths.size(); i++)
    {
      log(DEBUG) << paths[i];
      if(i < paths.size() - 1)
        log(DEBUG) << ":";
    }
    log(DEBUG) << endl;
  }
  
  _loadLibraries(paths);
}

void CameraSystem::_loadLibraries(const Vector<String> &paths)
{
  Vector<String> files;
  
  int numberOfLoadLibraries = 0;
  
  for(auto &path: paths)
  {
    getFiles(path, LIB_MATCH_STRING, files);
    
    for(auto &file: files)
    {
      std::ifstream ff(file, std::ios::binary);
      
      if(ff.good()) // File readable
      {
        ff.close();
        
        DepthCameraLibraryPtr p(new DepthCameraLibrary(file));
        
        if(!p->load())
        {
          log(WARNING) << "CameraSystem: Failed to load library " << file << ". Ignoring it." << endl;
          continue;
        }
        
        DepthCameraFactoryPtr factory = p->getDepthCameraFactory();
        
        if(!factory || !addDepthCameraFactory(factory))
        {
          log(WARNING) << "CameraSystem: Failed to load or register a depth camera factory from library " << file  <<". Ignoring this library." << endl;
          continue;
        }
        
        // Preserving this library
        _libraries.push_back(p);
        
        numberOfLoadLibraries++;
        
        log(INFO) << "CameraSystem: Successfully loaded factory from library " << file << endl;
      }
      else
        log(WARNING) << "CameraSystem: Library file " << file << " is not readable. Ignoring it." << endl;
    }
  }
  
  if(!numberOfLoadLibraries)
    log(WARNING) << "CameraSystem: No depth camera library found or loaded." << endl;
}


bool CameraSystem::addDepthCameraFactory(DepthCameraFactoryPtr factory)
{
  // Adding factory to the "_factories" map for later query
  const Vector<DevicePtr> &devices = factory->getSupportedDevices();
  
  if(!devices.size()) // No devices?
    return false;
  
  int numberOfDevicesAdded = 0;
  
  for(auto &device: devices)
  {
    if(device->serialNumber().size())
    {
      log(WARNING) << "CameraSystem: Device type " << device->id() << " being registered from factory has serial number. Ignoring it." << endl;
      continue;
    }
    
    auto f = _factories.find(device->id());
    if(f != _factories.end())
    {
      log(WARNING) << "CameraSystem: Device type " << device->id() << " already has a factory '" << f->second->name() << "'. Not overwriting it." << endl;
      continue;
    }
    
    numberOfDevicesAdded++;
      
    _factories[device->id()] = factory;
  }
  
  if(!numberOfDevicesAdded)
  {
    log(WARNING) << "CameraSystem: No devices added from factory '" << factory->name() << ". Ignoring this factory." << endl;
    return false;
  }
  
  return true;
}


Vector<DevicePtr> CameraSystem::scan()
{
  Vector<DevicePtr> devices = DeviceScanner::scan();
  
  Vector<DevicePtr> toReturn;
  
  for(auto &device: devices)
  {
    Device d(device->interface(), device->deviceID(), ""); // get device ID without serial number
    
    auto f = _factories.find(d.id());
    if(f != _factories.end())
      toReturn.push_back(device);
  }
  
  return toReturn;
}

DepthCameraPtr CameraSystem::connect(DevicePtr device)
{
  Device d(device->interface(), device->deviceID(), ""); // get device ID without serial number
  
  auto f = _factories.find(d.id());
  
  if(f != _factories.end())
    return f->second->getDepthCamera(device);
  else
    return 0;
}


  
}