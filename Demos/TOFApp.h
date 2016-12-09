/*! 
 * ============================================================================
 *
 * @addtogroup		TOFApp
 * @{
 *
 * @file		TOFApp.h
 * @version		1.0
 * @date		12/14/2015
 *
 * @note		Generalized TOF Application class
 * 
 * Copyright(c) 2007-2012 Texas Instruments Corporation, All Rights Reserved.
 * TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
 *
 * ============================================================================
 */
#include <deque>
#include <string>
#include <CameraSystem.h>
#include <Common.h>
#include <unistd.h>
#include <termio.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#ifndef __TOFAPP_H__
#define __TOFAPP_H__

using namespace std;
using namespace Voxel;
using namespace cv;

#define TOF_WIDTH		320
#define TOF_HEIGHT		240

class TOFApp
{
public:
   TOFApp();
   TOFApp(int w, int h);
   void Init(int w, int h);
   void setDim(int w, int h);
   DepthCameraPtr getDepthCamera();
   FrameSize &getDim();
   void setLoopDelay(int delay);
   int getLoopDelay();
   int getIllumPower();
   int getExposure();
   void setIllumPower(int power);
   void setExposure(int exposure);
   bool setProfile(Voxel::String name);
   Voxel::String getProfile();
   DepthCamera::FrameType getFrameType();
   bool isRunning();
   bool isConnected();
   void start();
   void stop();   
   bool connect(DepthCamera::FrameType frmType=DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME);
   void disconnect();
   void updateRegisters();
   void lock();
   void unlock();
   static void *eventLoop(void *app);
   virtual void update(Frame *frm) {}

private:
   pthread_t _thread;
   pthread_mutex_t _mtx;
   bool _isRunning;
   bool _isConnected;
   CameraSystem _sys;
   DepthCameraPtr _depthCamera;
   FrameSize _dimen;
   FrameRate _frate;
   int _illum_power;
   int _intg;
   int _loopDelay;
   Voxel::String _profile;
   DepthCamera::FrameType _frameType;
};

#endif // __AIRMOUSE_H__
/*! @} */

