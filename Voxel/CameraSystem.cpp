/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <CameraSystem.h>
#include <Logger.h>
#include <Configuration.h>
#include <Filter/VoxelFilterFactory.h>
#include "PointCloudFrameGenerator.h"

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
  
  tVector<String> paths;
  
  c.getLibPaths(paths);
  
  _loadLibraries(paths);
}

void CameraSystem::_loadLibraries(const tVector<String> &paths)
{
  tVector<String> files;
  
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
        
        if (p->getABIVersion() != VOXEL_ABI_VERSION)
        {
          logger(LOG_WARNING) << "CameraSystem: Ignoring Voxel library " << file << " with ABI version = " << p->getABIVersion() << ". Expected ABI version = " << VOXEL_ABI_VERSION << std::endl;
          continue;
        }
        
        if(!p->load())
        {
          logger(LOG_WARNING) << "CameraSystem: Failed to load library " << file << ". Ignoring it." << std::endl;
          continue;
        }
        
        bool success = false;
        
        DepthCameraFactoryPtr factory = p->getDepthCameraFactory();
        
        if(!factory || !addDepthCameraFactory(factory))
          logger(LOG_WARNING) << "CameraSystem: Failed to load or register a depth camera factory from library " << file  <<". Ignoring this library." << std::endl;
        else
          success = true;
        
        FilterFactoryPtr filterFactory = p->getFilterFactory();
        
        if(!filterFactory || !addFilterFactory(filterFactory))
          logger(LOG_DEBUG) << "CameraSystem: Could not find filter factory" << std::endl;
        else
          success = true;
        
        DownloaderFactoryPtr downloaderFactory = p->getDownloaderFactory();
        
        if(!downloaderFactory || !addDownloaderFactory(downloaderFactory))
          logger(LOG_DEBUG) << "CameraSystem: Could not find downloader factory" << std::endl;
        else
          success = true;
        
        if(success)
        {
          // Preserving this library
          _libraries.push_back(p);
          numberOfLoadLibraries++;
          logger(LOG_INFO) << "CameraSystem: Successfully loaded factory from library " << file << std::endl;
        }
      }
      else
        logger(LOG_WARNING) << "CameraSystem: Library file " << file << " is not readable. Ignoring it." << std::endl;
    }
  }
  
  if(!numberOfLoadLibraries)
    logger(LOG_WARNING) << "CameraSystem: No depth camera library found or loaded." << std::endl;
}


bool CameraSystem::addDepthCameraFactory(DepthCameraFactoryPtr factory)
{
  // Adding factory to the "_factories" map for later query
  const tVector<DevicePtr> &devices = factory->getSupportedDevices();
  
  if(!devices.size()) // No devices?
    return false;
  
  int numberOfDevicesAdded = 0;
  
  for(auto &device: devices)
  {
    if(device->serialNumber().size())
    {
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " being registered from factory has serial number. Ignoring it." << std::endl;
      continue;
    }
    
    auto f = _factories.find(device->id());
    if(f != _factories.end())
    {
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " already has a factory '" << f->second->name() << "'. Not overwriting it." << std::endl;
      continue;
    }
    
    numberOfDevicesAdded++;
      
    _factories[device->id()] = factory;
  }
  
  if(!numberOfDevicesAdded)
  {
    logger(LOG_WARNING) << "CameraSystem: No devices added from factory '" << factory->name() << "'. Ignoring this factory." << std::endl;
    return false;
  }
  
  tVector<GeneratorIDType> supportedGeneratorIDs = factory->getSupportedGeneratorTypes();
  
  for(auto &d: supportedGeneratorIDs)
  {
    auto f = _factoryForGeneratorID.find(d);
    
    if(f != _factoryForGeneratorID.end())
    {
      logger(LOG_DEBUG) << "CameraSystem: A factory is already registered for generator ID '" << d << "'. Not replacing it." << std::endl;
      continue;
    }
    
    _factoryForGeneratorID[d] = factory;
  }
  
  return true;
}

bool CameraSystem::getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator)
{
  if(frameType == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
  {
    frameGenerator = FrameGeneratorPtr(new PointCloudFrameGenerator());
    return true;
  }
  else
  {
    auto f = _factoryForGeneratorID.find(generatorID);
    
    if(f == _factoryForGeneratorID.end())
    {
      logger(LOG_ERROR) << "CameraSystem: Could not find factory for generator ID '" << generatorID << "'" << std::endl;
      return false;
    }
    return f->second->getFrameGenerator(frameType, generatorID, frameGenerator);
  }
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

tVector<String> CameraSystem::getSupportedFilters()
{
  tVector<String> f;
  f.reserve(_filterFactories.size());
  
  for(auto &x: _filterFactories)
    f.push_back(x.first);
  
  return f;
}

bool CameraSystem::getFilterDescription(const String &filterName, FilterDescription &description)
{
  auto x = _filterFactories.find(filterName);
  
  if(x != _filterFactories.end())
  {
    auto &desc = x->second->getSupportedFilters();
    
    auto y = desc.find(filterName);
    
    if(y != desc.end())
    {
      description = y->second;
      return true;
    }
  }
  return false;
}


tVector<DevicePtr> CameraSystem::scan()
{
  tVector<DevicePtr> devices = DeviceScanner::scan();
  
  tVector<DevicePtr> toReturn;
  
  tVector<int> channels;
  
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
        const tVector<DevicePtr> &ds = device->getDevices(channels);
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
    if(!c->second->refreshParams())
      logger(LOG_ERROR) << "CameraSystem: Could not refresh parameters for " << c->second->id() << "." << std::endl;
    else
      logger(LOG_INFO) << "CameraSystem: Successfully refreshed parameters for " << c->second->id() << "." << std::endl;
    return c->second;
  }
  
  Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
  
  auto f = _factories.find(d.id());
  
  if(f != _factories.end())
  {
    DepthCameraPtr p = f->second->getDepthCamera(device);
    if(!p->refreshParams())
      logger(LOG_ERROR) << "CameraSystem: Could not refresh parameters for " << p->id() << "." << std::endl;
    else
      logger(LOG_INFO) << "CameraSystem: Successfully refreshed parameters for " << p->id() << "." << std::endl;
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
    
    bool ret;
    if(reset)
      ret = depthCamera->reset();
    
    return depthCamera->close() && ret;
  }
  
  return false;
}

tVector<DevicePtr> CameraSystem::getProgrammableDevices()
{
  tVector<DevicePtr> devices = DeviceScanner::scan();
  
  tVector<DevicePtr> toReturn;
  
  for(auto &device: devices)
  {
    Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
    
    auto f = _downloaderFactories.find(d.id());
    if(f != _downloaderFactories.end())
    {
      toReturn.push_back(device);
    }
  }
  
  return toReturn;
}

bool CameraSystem::addDownloaderFactory(DownloaderFactoryPtr factory)
{
  // Adding factory to the "_downloaderFactories" map for later query
  const tVector<DevicePtr> &devices = factory->getSupportedDevices();
  
  if(!devices.size()) // No devices?
    return false;
  
  int numberOfDevicesAdded = 0;
  
  for(auto &device: devices)
  {
    if(device->serialNumber().size())
    {
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " being registered from downloader factory has serial number. Ignoring it." << std::endl;
      continue;
    }
    
    auto f = _downloaderFactories.find(device->id());
    if(f != _downloaderFactories.end())
    {
      logger(LOG_WARNING) << "CameraSystem: Device type " << device->id() << " already has a downloader factory '" << f->second->name() << "'. Not overwriting it." << std::endl;
      continue;
    }
    
    numberOfDevicesAdded++;
    
    _downloaderFactories[device->id()] = factory;
  }
  
  if(!numberOfDevicesAdded)
  {
    logger(LOG_WARNING) << "CameraSystem: No devices added from downloader factory '" << factory->name() << ". Ignoring this factory." << std::endl;
    return false;
  }
  
  return true;
}

DownloaderPtr CameraSystem::getDownloader(const DevicePtr &device)
{
  Device d(device->interfaceID(), device->deviceID(), ""); // get device ID without serial number
  
  auto f = _downloaderFactories.find(d.id());
  
  if(f != _downloaderFactories.end())
  {
    DownloaderPtr p = f->second->getDownloader(device);
    return p;
  }
  else
    return nullptr;
}


CameraSystem::~CameraSystem()
{
  _depthCameras.clear();
  _factories.clear();
  _downloaderFactories.clear();
  _factoryForGeneratorID.clear();
  _libraries.clear();
}

  
}