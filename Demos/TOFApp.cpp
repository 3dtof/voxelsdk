/*! 
 * ============================================================================
 *
 * @addtogroup		TOFApp
 * @{
 *
 * @file		TOFApp.cpp
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
#define __TOFAPP_CPP__

#include "TOFApp.h"

#define FRAME_QUEUE_SZ		3

// Frame callback
static deque<Voxel::Frame *> qFrame; 
static pthread_mutex_t gmtx;

static void frameCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
   pthread_mutex_lock(&gmtx);
   if (qFrame.size() < FRAME_QUEUE_SZ) {
      if (c == DepthCamera::FRAME_DEPTH_FRAME) {
         const Voxel::DepthFrame *f = dynamic_cast<const Voxel::DepthFrame *>(&frame);
         Voxel::Frame *nf = dynamic_cast<Voxel::Frame *>(new Voxel::DepthFrame(*f));                    
         qFrame.push_back(nf);
      }
      else if (c == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) {
	 const Voxel::XYZIPointCloudFrame *f = dynamic_cast<const Voxel::XYZIPointCloudFrame *>(&frame);
         Voxel::Frame *nf = dynamic_cast<Voxel::Frame *>(new Voxel::XYZIPointCloudFrame(*f));                    
         qFrame.push_back(nf);
      }
   }
   pthread_mutex_unlock(&gmtx);
}


// Accessors
void TOFApp::setIllumPower(int power) 
{ 
   _illum_power = power; 
}

void TOFApp::setExposure(int exposure) 
{
   _intg = exposure;
}

void TOFApp::setDim(int w, int h) 
{
   _dimen.width=w; 
   _dimen.height=h;
}

DepthCameraPtr TOFApp::getDepthCamera() 
{
   return _depthCamera;
}

FrameSize &TOFApp::getDim() 
{
   return _dimen;
}

void TOFApp::setLoopDelay(int delay) 
{
   _loopDelay = delay;
}

int TOFApp::getLoopDelay() 
{
   return _loopDelay;
}

int TOFApp::getIllumPower() 
{ 
   return _illum_power; 
}

int TOFApp::getExposure() 
{ 
   return _intg; 
}

bool TOFApp::setProfile(Voxel::String name)
{
   bool rc = false;
   const Map<int, Voxel::String> &profiles = _depthCamera->getCameraProfileNames();
   for (auto &p: profiles) {
      if (p.second == name) {
         int profile_id = p.first;
         ConfigurationFile *c = _depthCamera->configFile.getCameraProfile(p.first);
         if (c && c->getLocation() == ConfigurationFile::IN_CAMERA) {
            if (_depthCamera->setCameraProfile(profile_id)) {
              rc = true;
              break;
            }
         }
      }
   }
   return rc;
}

Voxel::String TOFApp::getProfile()
{
   return _profile;
}


DepthCamera::FrameType TOFApp::getFrameType()
{
   return _frameType;
}

bool TOFApp::isRunning() 
{
   return _isRunning;
}

bool TOFApp::isConnected() 
{
   return _isConnected;
}


void *TOFApp::eventLoop(void *p)
{
   bool done = false;
   bool empty;
   TOFApp *app = (TOFApp *)p;
   logger.setDefaultLogLevel(LOG_INFO);   

   if (!app->isConnected()) 
      goto err_exit;

   app->_isRunning = true;
   while (!done) {

      pthread_mutex_lock(&gmtx);
      empty = qFrame.empty();      
      pthread_mutex_unlock(&gmtx);
 
      if (!empty) {

         pthread_mutex_lock(&gmtx);
         Voxel::Frame *frm = qFrame.front(); 
         pthread_mutex_unlock(&gmtx);
    
         app->update(frm);
         delete frm;

         pthread_mutex_lock(&gmtx);
         qFrame.pop_front();
         pthread_mutex_unlock(&gmtx);
      }

      done = !app->_isRunning;   
#ifdef TOF_INTERACTIVE
      waitKey(app->_loopDelay);
#else
      usleep(10000);
#endif
   }
   
   app->disconnect();

err_exit:
   pthread_exit(NULL);
}


TOFApp::TOFApp()
{
   Init(TOF_WIDTH, TOF_HEIGHT);
}

TOFApp::TOFApp(int w, int h)
{
   Init(w, h);
}

void TOFApp::Init(int w, int h)
{
   _isRunning = false;
   _isConnected = false;
   _dimen.width = w;
   _dimen.height = h;
   _frate.numerator = 30;
   _frate.denominator = 1;
   _loopDelay = 66;
   _illum_power = 60;
   _intg = 20;
   _profile = "MetrilusLongRange";
   //_profile = "LongRangeHW";
}


bool TOFApp::connect(DepthCamera::FrameType frmType)
{
   const vector<DevicePtr> &devices = _sys.scan();
   if (devices.size() > 0) {
      _depthCamera = _sys.connect(devices[0]);
      if (!_depthCamera) 
         return false; 
      if (!_depthCamera->isInitialized()) 
         return false;
   }
   else 
      return false;

   #if 0   //Enable this to list all the profiles that are supported by the camera
   cout << "List of Profiles:" << endl;
   auto &names = _depthCamera->configFile.getCameraProfileNames();
  
   for(auto &n: names)
   {
      cout << n.first << " -> " << n.second;

      auto c = _depthCamera->configFile.getCameraProfile(n.first);
      if(c->getLocation() == ConfigurationFile::IN_CAMERA)
      cout << " (HW)";
    
      if(n.first == _depthCamera->configFile.getDefaultCameraProfileIDInCamera() || 
         n.first == _depthCamera->configFile.getDefaultCameraProfileIDInHost())
         cout << " (DEFAULT)";
    
      if(n.first == _depthCamera->getCurrentCameraProfileID())
         cout << " (ACTIVE)";
    
      cout << endl;
   }
   #endif
              
   if (setProfile(_profile)) 
      cout << "Profile " << _profile << " found." << endl;
   else 
      cout << "Profile " << _profile << "not found." << endl;
   updateRegisters();

   _frameType = frmType;
   _depthCamera->registerCallback(_frameType, frameCallback);
   _depthCamera->setFrameSize(_dimen);
   _depthCamera->setFrameRate(_frate);

   VideoMode m;
  
   if(_depthCamera->getFrameSize(m.frameSize) && _depthCamera->getFrameRate(m.frameRate))
   {
      cout << "Current video mode: " << m.frameSize.width << "X" << m.frameSize.height << "@" << m.getFrameRate() << "fps\n";
   }

   _depthCamera->start();
   _isConnected = true;

   return true;
}


void TOFApp::start()
{
   if (!_isRunning) 
      pthread_create(&_thread, NULL, &TOFApp::eventLoop, this);
}

void TOFApp::stop()
{      
   if (_isRunning) {
      _isRunning = false;
      pthread_join(_thread, NULL);
   }
}

void TOFApp::updateRegisters()
{
   _depthCamera->set("illum_power_percentage", (uint)_illum_power);
   _depthCamera->set("intg_duty_cycle", (uint)_intg);
}

void TOFApp::disconnect()
{
   _depthCamera->stop();
}

void TOFApp::lock()
{
   pthread_mutex_lock(&_mtx);
}

void TOFApp::unlock()
{
   pthread_mutex_unlock(&_mtx);
}


#undef __JIVE_CPP__
/*! @} */
