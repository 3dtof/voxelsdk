#include <pcl/visualization/cloud_viewer.h>
#include "PCLGrabber.h"

#include <CameraSystem.h>
 
namespace Voxel
{
  
class SimplePCLViewer
{
  Voxel::CameraSystem _sys;
  Voxel::DepthCameraPtr _depthCamera;
public:
  SimplePCLViewer(): _viewer("PCL Voxel Viewer") 
  {
    const Vector<DevicePtr> &devices = _sys.scan();
    
    if(devices.size() > 0)
      _depthCamera = _sys.connect(devices[0]); // Connect to first available device
    else
      logger(ERROR) << "SimplePCLViewer: Could not find a compatible device." << std::endl;
  }
  
  bool firstTime = true;
  
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud)
  {
    if (!_viewer.wasStopped())
     _viewer.showCloud (cloud);
    
    if(firstTime)
    {
      _viewer.runOnVisualizationThreadOnce([](pcl::visualization::PCLVisualizer &viz){
        viz.setCameraPosition(15, 15, 20, 0, 0, 5, 0, 0, 1);
      });
      firstTime = false;
    }
  }
  
  void run()
  {
    if(!_depthCamera)
      return;
    
    pcl::Grabber* interface = new Voxel::PCLGrabber(*_depthCamera);
    
    boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> f =
    boost::bind (&SimplePCLViewer::cloud_cb_, this, _1);
    
    interface->registerCallback(f);
    
    interface->start();
    
    while (!_viewer.wasStopped())
    {
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
    
    interface->stop();
  }
  
protected:
  pcl::visualization::CloudViewer _viewer;
};

}

int main ()
{
  Voxel::logger.setDefaultLogLevel(Voxel::INFO);
  Voxel::SimplePCLViewer v;
  v.run();
  return 0;
}