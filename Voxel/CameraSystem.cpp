/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <CameraSystem.h>
#include <Logger.h>
#include <Configuration.h>
#include <Filter/VoxelFilterFactory.h>

#include <fstream>

#ifdef LINUX
#define LIB_MATCH_STRING ".so"
#elif defined(WINDOWS)
#define LIB_MATCH_STRING ".dll"
#endif

namespace Voxel
{
  
CameraSystem::CameraSystem()
{
  _init();
}


void CameraSystem::_init()
{
  addFilterFactory(FilterFactoryPtr(new VoxelFilterFactory()));
  
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
        
        if (p->getABIVersion() != VOXEL_ABI_VERISON)
        {
          logger(LOG_WARNING) << "CameraSystem: Ignoring Voxel library " << file << " with ABI version = " << p->getABIVersion() << ". Expected ABI version = " << VOXEL_ABI_VERISON << std::endl;
          continue;
        }
        
        bool success = false;
        
        DepthCameraFactoryPtr factory = p->getDepthCameraFactory();
        
        if(!factory || !addDepthCameraFactory(factory))
          logger(LOG_WARNING) << "CameraSystem: Failed to load or register a depth camera factory from library " << file  <<". Ignoring this library." << endl;
        else
          success = true;
        
        FilterFactoryPtr filterFactory = p->getFilterFactory();
        
        if(!filterFactory || !addFilterFactory(filterFactory))
          logger(LOG_WARNING) << "CameraSystem: Failed to load or register filter factory" << std::endl;
        else
          success = true;
        
        if(success)
        {
          // Preserving this library
          _libraries.push_back(p);
          numberOfLoadLibraries++;
          logger(LOG_INFO) << "CameraSystem: Successfully loaded factory from library " << file << endl;
        }
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

bool CameraSystem::addFilterFactory(FilterFactoryPtr filterFactory)
{
  auto &desc = filterFactory->getSupportedFilters();
  
  for(auto &d: desc)
  {
    auto x = _filterFactories.find(d.first);
    
    if(x != _filterFactories.end())
    {
      logger(LOG_WARNING) << "CameraSystem: A filter with name '" << d.first << "' is already registered. Ignoring the new one." << std::endl;
      continue;
    }
    
    _filterFactories[d.first] = filterFactory;
  }
  return true;
}

Vector<String> CameraSystem::getSupportedFilters()
{
  Vector<String> f;
  f.reserve(_filterFactories.size());
  
  for(auto &x: _filterFactories)
    f.push_back(x.first);
  
  return f;
}


Vector<DevicePtr> CameraSystem::scan()
{
  Vector<DevicePtr> devices = DeviceScanner::scan();
  
  Vector<DevicePtr> toReturn;
  
  Vector<int> channels;
  
  for(auto &device: devices)
  {
    Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
    
    auto f = _factories.find(d.id());
    if(f != _factories.end())
    {
      if(!f->second->getChannels(d, channels) || channels.size() == 0)
      {
        logger(LOG_WARNING) << "CameraSystem: Could not get channels for device '" << device->id() << "'" << std::endl;
        continue;
      }
      
      if(channels.size() == 1)
        toReturn.push_back(device);
      else
      {
        const Vector<DevicePtr> &ds = device->getDevices(channels);
        toReturn.insert(toReturn.end(), ds.begin(), ds.end());
      }
    }
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
    _depthCameras[p->getDevice()->id()] = p;
    return p;
  }
  else
    return nullptr;
}


FilterPtr CameraSystem::createFilter(const String &name, DepthCamera::FrameType type)
{
  auto f = _filterFactories.find(name);
  
  if(f != _filterFactories.end())
  {
    return f->second->createFilter(name, type);
  }
  else
  {
    logger(LOG_ERROR) << "CameraSystem: Unknown filter '" << name << "'" << std::endl;
    return nullptr;
  }
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