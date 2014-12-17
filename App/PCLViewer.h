/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PCLVIEWER_H
#define VOXEL_PCLVIEWER_H

#include <DepthCamera.h>
#include <boost/shared_ptr.hpp>

/// Forward declaration of PCL related classes
namespace pcl
{
namespace visualization
{
class CloudViewer;
}

template <typename T>
class PointCloud;

class PointXYZI;

class Grabber;
}

namespace Voxel
{

class PCLViewer
{
protected:
  Voxel::DepthCameraPtr _depthCamera;
  
  Ptr<pcl::visualization::CloudViewer> _viewer;
  Ptr<pcl::Grabber> _grabber; // This will link to our Voxel::PCLGrabber
  
  bool _firstRun = true;
  void _cloudRenderCallback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &cloud);
  
public:
  PCLViewer();
  
  void setDepthCamera(Voxel::DepthCameraPtr depthCamera);
  
  void start();
  bool isRunning();
  void stop();
  
  virtual ~PCLViewer() 
  {
    if(isRunning()) stop();
  }
};


}
#endif // PCLVIEWER_H
