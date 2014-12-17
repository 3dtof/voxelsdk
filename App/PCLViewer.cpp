/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "PCLViewer.h"
#include <pcl/visualization/cloud_viewer.h>
#include "PCLGrabber.h"
#include <vtkRenderWindow.h>

namespace Voxel
{

PCLViewer::PCLViewer(): _viewer(Ptr<pcl::visualization::CloudViewer>(new pcl::visualization::CloudViewer("PCL Voxel Viewer")))
{
}

void PCLViewer::setDepthCamera(DepthCameraPtr depthCamera)
{
  if(isRunning())
  {
    stop();
    _depthCamera = depthCamera;
    start();
  }
  else
    _depthCamera = depthCamera;
}

void PCLViewer::_cloudRenderCallback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &cloud)
{
  if (!_viewer->wasStopped())
    _viewer->showCloud(cloud);
  
  if(_firstRun)
  {
    _viewer->runOnVisualizationThreadOnce([](pcl::visualization::PCLVisualizer &viz){
      viz.setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
    });
    _firstRun = false;
  }
}

void PCLViewer::start()
{
  if(!_depthCamera)
    return;
  
  if(!_viewer or _viewer->wasStopped())
  {
    _viewer = Ptr<pcl::visualization::CloudViewer>(new pcl::visualization::CloudViewer("PCL Voxel Viewer"));
  }
  
  _firstRun = true;
  
  _grabber = Ptr<pcl::Grabber>(new Voxel::PCLGrabber(*_depthCamera));
  
  boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> f = boost::bind(&PCLViewer::_cloudRenderCallback, this, _1);
  
  _grabber->registerCallback(f);
  _grabber->start();
}

bool PCLViewer::isRunning()
{
  return _grabber and _grabber->isRunning() and !viewerStopped();
}

bool PCLViewer::viewerStopped()
{
  return !_viewer or _viewer->wasStopped();
}

void PCLViewer::stop()
{
  if(_grabber)
  {
    _grabber = nullptr;
    
    if(_viewer and _viewer->wasStopped())
      _viewer = nullptr;
  }
}

}