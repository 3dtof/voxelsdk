/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CameraSystem.h"
#include <Logger.h>
#include "Configuration.h"

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
  Configuration c;
  
  Vector<String> paths;
  
  c.getLibPaths(paths);
  
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
          logger(LOG_WARNING) << "CameraSystem: Failed to load library " << file << ". Ignoring it." << endl;
          continue;
        }
        
        DepthCameraFactoryPtr factory = p->getDepthCameraFactory();
        
        if(!factory || !addDepthCameraFactory(factory))
        {
          logger(LOG_WARNING) << "CameraSystem: Failed to load or register a depth camera factory from library " << file  <<". Ignoring this library." << endl;
          continue;
        }
        
        // Preserving this library
        _libraries.push_back(p);
        
        numberOfLoadLibraries++;
        
        logger(LOG_INFO) << "CameraSystem: Successfully loaded factory from library " << file << endl;
      }
      else
        logger(LOG_WARNING) << "CameraSystem: Library file " << file << " is not readable. Ignoring it." << endl;
    }
  }
  
  if(!numberOfLoadLibraries)
    logger(LOG_WARNING) << "CameraSystem: No depth camera library found or loaded." << endl;
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
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " being registered from factory has serial number. Ignoring it." << endl;
      continue;
    }
    
    auto f = _factories.find(device->id());
    if(f != _factories.end())
    {
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " already has a factory '" << f->second->name() << "'. Not overwriting it." << endl;
      continue;
    }
    
    numberOfDevicesAdded++;
      
    _factories[device->id()] = factory;
  }
  
  if(!numberOfDevicesAdded)
  {
    logger(LOG_WARNING) << "CameraSystem: No devices added from factory '" << factory->name() << ". Ignoring this factory." << endl;
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
    Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
    
    auto f = _factories.find(d.id());
    if(f != _factories.end())
      toReturn.push_back(device);
  }
  
  return toReturn;
}

DepthCameraPtr CameraSystem::connect(const DevicePtr &device)
{
  auto c = _depthCameras.find(device->id());
  
  if(c != _depthCameras.end())
  {
    logger(LOG_INFO) << "CameraSystem: DepthCamera for " << device->id() << " was already created. Returning it." << std::endl;
    return c->second;
  }
  
  Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
  
  auto f = _factories.find(d.id());
  
  if(f != _factories.end())
  {
    DepthCameraPtr p = f->second->getDepthCamera(device);
    _depthCameras[device->id()] = p;
    return p;
  }
  else
    return 0;
}

bool CameraSystem::disconnect(const DepthCameraPtr &depthCamera, bool reset)
{
  auto f = _depthCameras.find(depthCamera->getDevice()->id());
  
  if(f != _depthCameras.end())
  {
    _depthCameras.erase(f);
     
    if(reset)
      return depthCamera->reset();
    return true;
  }
  
  return false;
}


CameraSystem::~CameraSystem()
{
  _depthCameras.clear();
  _factories.clear();
  _libraries.clear();
}

  
}