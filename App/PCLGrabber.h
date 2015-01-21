/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PCLGRABBER_H
#define VOXEL_PCLGRABBER_H

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <DepthCamera.h>

#include "VoxelPCLExports.h"

namespace Voxel
{

class VOXELPCL_EXPORT PCLGrabber: public pcl::Grabber
{
public:
  typedef void (PointCloudCallBack) (const pcl::PointCloud<pcl::PointXYZI> &);
  typedef void (PointCloudFrameCallBack) (const Voxel::PointCloudFrame &);
  typedef void (DepthImageCallBack) (const Voxel::DepthFrame &);
  typedef void (RawImageCallBack) (const Voxel::RawFrame &, const Voxel::DepthCamera::FrameType type); 
  // For ToF type depth cameras, this could return reference to Voxel::ToFRawFrame or Voxel::RawDataFrame base on the 'type'
  
protected:
  DepthCamera &_depthCamera;
  
  FrameBufferManager<pcl::PointCloud<pcl::PointXYZI>> _pointCloudBuffer;
  
  boost::signals2::signal<PointCloudFrameCallBack>* _pointCloudFrameSignal;
  boost::signals2::signal<PointCloudCallBack>* _pointCloudSignal;
  boost::signals2::signal<DepthImageCallBack>* _depthImageSignal;
  boost::signals2::signal<RawImageCallBack>* _rawImageSignal;
  
  void _callback(DepthCamera &depthCamera, const Frame &frame, DepthCamera::FrameType type);
  
public:
  PCLGrabber(DepthCamera &depthCamera);
  
  virtual float getFramesPerSecond() const 
  { 
    FrameRate r;
    if(!_depthCamera.getFrameRate(r))
      return 0;
    else
      return r.getFrameRate();
  }
  
  virtual std::string getName() const { return _depthCamera.id(); }
  
  virtual bool isRunning() const { return _depthCamera.isRunning(); }
  
  virtual void start() { _depthCamera.start(); }
  
  virtual void stop() { _depthCamera.stop(); _depthCamera.wait(); }
  
  virtual ~PCLGrabber() 
  {
    if(isRunning())
      stop();
  }
};

/// Support for std::shared_ptr <-> boost::shared_ptr
template<typename T>
boost::shared_ptr<T> make_shared_ptr(std::shared_ptr<T>& ptr)
{
  return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

template<typename T>
std::shared_ptr<T> make_shared_ptr(boost::shared_ptr<T>& ptr)
{
  return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

}

#endif // PCLGRABBER_H
