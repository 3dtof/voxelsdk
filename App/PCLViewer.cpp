/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "PCLViewer.h"
#include "PCLGrabber.h"

#define PCL_NO_PRECOMPILE
#include <pcl/visualization/pcl_visualizer.h>
#undef PCL_NO_PRECOMPILE

namespace Voxel
{
  
#define CLOUD_NAME "cloud"

PCLViewer::PCLViewer()
{
}

void PCLViewer::_renderLoop()
{
  if(!_viewer)
  {
    _viewer =  Ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("PCL Voxel Viewer"));
    _viewer->setBackgroundColor(0, 0, 0);
    //_viewer->createInteractor();
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
    _viewer->setShowFPS(false);
    _viewer->resetCameraViewpoint();
    _viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
  }
  
  if(_viewer->wasStopped())
    _viewer->resetStoppedFlag();
  
  bool firstTime = false;
  
  while(!_stopLoop && !_viewer->wasStopped())
  {
    {
      Lock<Mutex> _(_cloudUpdateMutex);
      
      if(_cloud && _handler)
      {
        double psize = 1.0, opacity = 1.0, linesize =1.0;
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        _viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
        
        if(!_viewer->updatePointCloud(_cloud, *_handler, CLOUD_NAME))
        {
          _viewer->addPointCloud(_cloud, *_handler, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CLOUD_NAME);
        }
        else
        {
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, CLOUD_NAME);
          _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, CLOUD_NAME);
        }
      }
    }
    
    _viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::microseconds(10000));
    
    if(firstTime)
    {
      _viewer->setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
      firstTime = false;
    }
  }
  
  //_viewer->close();
  //_viewer = nullptr;
  _stopLoop = true;
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

void PCLViewer::removeDepthCamera()
{
  if(isRunning())
  {
    stop();
  }
  
  _depthCamera = nullptr;
}


void PCLViewer::_cloudRenderCallback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &cloud)
{
  if(_viewer && !_viewer->wasStopped())
  {
    Lock<Mutex> _(_cloudUpdateMutex);
    _cloud = boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>(cloud);
    _handler = Ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>>(
      new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(_cloud, "intensity"));
  }
}

void PCLViewer::start()
{
  if(!_depthCamera)
    return;
  
  _stopLoop = true;
  
  if(_renderThread.joinable())
    _renderThread.join();
  
  _stopLoop = false;
  
  _renderThread = std::thread(&PCLViewer::_renderLoop, this);
    
  _grabber = Ptr<pcl::Grabber>(new Voxel::PCLGrabber(*_depthCamera));
  
  boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> f = boost::bind(&PCLViewer::_cloudRenderCallback, this, _1);
  
  _grabber->registerCallback(f);
  _grabber->start();
}

bool PCLViewer::isRunning()
{
  return _grabber && _grabber->isRunning() && !_stopLoop && !viewerStopped();
}

bool PCLViewer::viewerStopped()
{
  return !_viewer || _viewer->wasStopped();
}

void PCLViewer::stop()
{
  _stopLoop = true;
  
  if(_renderThread.joinable()) _renderThread.join();
  
  if(_viewer) _viewer->removePointCloud(CLOUD_NAME);
  
  _grabber = nullptr;
  
  
}

}