/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PCLVIEWER_H
#define VOXEL_PCLVIEWER_H

#include <DepthCamera.h>
#include <boost/shared_ptr.hpp>

#include "VoxelPCLExports.h"

/// Forward declaration of PCL related classes
namespace pcl
{
namespace visualization
{
class PCLVisualizer;

template <typename T>
class PointCloudColorHandlerGenericField;
}

template <typename T>
class PointCloud;

struct PointXYZI;

class Grabber;
}

namespace Voxel
{

class VOXELPCL_EXPORT PCLViewer
{
protected:
  DepthCameraPtr _depthCamera;
  
  Mutex _cloudUpdateMutex;
  
  Ptr<pcl::visualization::PCLVisualizer> _viewer;
  Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> _handler;
  
  bool _stopLoop = false;
  
  Ptr<pcl::Grabber> _grabber; // This will link to our Voxel::PCLGrabber
  
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> _cloud;
  
  void _cloudRenderCallback(const pcl::PointCloud<pcl::PointXYZI> &cloud);
  
  void _renderLoop();
  
  std::thread _renderThread;
  
public:
  PCLViewer();
  
  void setDepthCamera(Voxel::DepthCameraPtr depthCamera);
  void removeDepthCamera();
  
  void start();
  bool isRunning();
  void stop();
  
  bool viewerStopped();
  
  virtual ~PCLViewer() 
  {
    if(isRunning()) stop();
    
    if(_renderThread.joinable()) _renderThread.join();
  }
};


}
#endif // PCLVIEWER_H
