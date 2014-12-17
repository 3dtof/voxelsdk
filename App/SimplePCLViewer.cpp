#include <CameraSystem.h>
#include <Common.h>

#include <thread>

#include "PCLViewer.h"
 
int main ()
{
  Voxel::logger.setDefaultLogLevel(Voxel::INFO);
  
  Voxel::CameraSystem _sys;
  Voxel::DepthCameraPtr _depthCamera;
  
  const Voxel::Vector<Voxel::DevicePtr> &devices = _sys.scan();
  
  if(devices.size() > 0)
    _depthCamera = _sys.connect(devices[0]); // Connect to first available device
  else
  {
    std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
    return -1;
  }
  
  if(!_depthCamera)
  {
    std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
    return -1;
  }
    
  Voxel::PCLViewer v;
  
  v.setDepthCamera(_depthCamera);
  
  v.start();
  
  while(v.isRunning())
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  return 0;
}