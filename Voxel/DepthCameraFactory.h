/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DEPTHCAMERA_FACTORY_H
#define VOXEL_DEPTHCAMERA_FACTORY_H

#include "DepthCamera.h"
#include "Common.h"
#include "Device.h"

namespace Voxel
{
  
/**
 *  \addtogroup CamSys
 *  @{
 */
class VOXEL_EXPORT DepthCameraFactory
{
protected:
  Vector<DevicePtr> _supportedDevices;
  
  inline void _addSupportedDevices(const Vector<DevicePtr> &devices);
  
  String _name;
  
public:
  DepthCameraFactory(const String &name): _name(name) {}
  
  inline const String &name() const { return _name; }
  
  inline const Vector<DevicePtr> &getSupportedDevices() const { return _supportedDevices; }
  
  // Get the list of channels supported by this device
  virtual bool getChannels(Device &device, Vector<int> &channels) = 0;
  
  // Instantiate a depth camera for the specified device
  virtual DepthCameraPtr getDepthCamera(DevicePtr device) = 0;
  
  virtual bool getFrameGenerator(uint8_t frameType, GeneratorIDType generatorID, FrameGeneratorPtr &frameGenerator) = 0;
  virtual Vector<GeneratorIDType> getSupportedGeneratorTypes() = 0; 
  
  virtual ~DepthCameraFactory() {}
};

void DepthCameraFactory::_addSupportedDevices(const Vector<DevicePtr> &devices)
{
  _supportedDevices.reserve(_supportedDevices.size() + devices.size());
  _supportedDevices.insert(_supportedDevices.end(), devices.begin(), devices.end());
}


// Implement this function in every device-specific voxel library

typedef Ptr<DepthCameraFactory> DepthCameraFactoryPtr;

#ifndef SWIG
extern "C" void getDepthCameraFactory(DepthCameraFactoryPtr &depthCameraFactory);

extern "C" int getABIVersion();
#endif

typedef void (*GetDepthCameraFactory)(DepthCameraFactoryPtr &depthCameraFactory); // Function type to return DepthCameraFactory

typedef int(*GetABIVersion)(); // Function type to return ABI version

/**
 * @}
 */
}

#endif // DEPTHCAMERA_FACTORY_H
