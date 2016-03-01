/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "USBBulkStreamer.h"

#include "FrameBuffer.h"
#include "Logger.h"
#include "USBSystem.h"
#include <USBIO.h>

// Ask for an extra 255 bytes of data to ensure we don't end on a 512 byte packet
#define BULK_XFER_EXTRA_SIZE  255
#define MAX_BUFFER_COUNT      2

namespace Voxel
{

class USBBulkStreamer::USBBulkStreamerPrivate
{
public:
  bool initialized = true;

  USBIOPtr usbIO;

  Ptr<Thread> captureThread;
  bool captureRunning = false;

  long transferred = 0, bufSize = 0;
  Vector<uint8_t> usbBuffer;
  
  unsigned int validFrames = 0, droppedFrames = 0, resetCount = 0, retriesCount = 0;

  uint8_t endpoint;

  long timeout = 200;

  USBBulkStreamerPrivate(USBIOPtr &usbIO, DevicePtr device, uint8_t endpoint) : _rawBuffers(10), usbIO(usbIO), endpoint(endpoint), bufSize(0)
  {
  }

  virtual ~USBBulkStreamerPrivate()
  {
    if (captureThread && captureThread->joinable())
      captureThread->join();
  }

  bool start()
  {
    //logger(LOG_INFO) << "USBBulkStreamer: Running Start" << std::endl;

    if(!usbIO->isInitialized())
    {
      logger(LOG_ERROR) << "USBBulkStreamer: USBIO not initialized" << std::endl;
      return false;
    }
    
    if(!captureRunning)
    {
      validFrames = droppedFrames = resetCount = retriesCount = 0;
      
      captureThread = Ptr<Thread> (new Thread(&USBBulkStreamerPrivate::run, this));
      captureRunning = true;
      return true;
    }

    return false;
  }

  bool stop()
  {
    //logger(LOG_INFO) << "USBBulkStreamer: Running Stop" << std::endl;
    if(captureRunning)
    {
      captureRunning = false;
      captureThread->join();
      return true;
    }
    return false;
  }

  void run()
  {
    //logger(LOG_INFO) << "USBBulkStreamer: Running Capture" << std::endl;
    if (!usbIO->resetBulkEndPoint(endpoint))
    {
      logger(LOG_ERROR) << "USBBulkStreamer: Bulk End point reset failed" << std::endl;
      return;
    }

    while(captureRunning)
    {
      if(usbIO->bulkTransfer(endpoint, usbBuffer.data(), bufSize + BULK_XFER_EXTRA_SIZE, transferred, timeout))
      {
        if(transferred == bufSize)
        {
          validFrames++;
          retriesCount = 0;
          putBuffer();
        }
        else
        {
          droppedFrames++;
          if(droppedFrames % 100 == 0)
            logger(LOG_ERROR) << "USBBulkStreamer: Dropped frames " << droppedFrames
                              << " Valid frames " << validFrames << std::endl;
        }
      }
      else
      {
        retriesCount++;
        if(retriesCount == 3)
        {
          if (!usbIO->resetBulkEndPoint(endpoint))
          {
            logger(LOG_ERROR) << "USBBulkStreamer: Bulk End point reset failed" << std::endl;
            captureRunning = false;
          }

          logger(LOG_WARNING) << "USBBulkStreamer: Resetting bulk endpoint at " << _timer.getCurrentRealTime() << std::endl;
          retriesCount = 0;
          resetCount++;
          if(resetCount % 100 == 0)
          {
            logger(LOG_ERROR) << "USBBulkStreamer: Did not get a frame in 100 attempts" << std::endl;
            captureRunning = false;
          }
        }
      }
    }
  }

protected:
  TimeStampType _sampleStart = 0;
  Timer _timer;

  FrameBufferManager<RawDataFrame> _rawBuffers;

  List<FrameBuffer<RawDataFrame>> _inUseBuffers;

  Mutex _dataAccessMutex;
  ConditionVariable _dataAvailableCondition;

public:
  
  bool setBufferSize(size_t bufSize)
  {
    if(captureRunning)
      return false;
    
    this->bufSize = bufSize;
    
    this->usbBuffer.resize(bufSize + BULK_XFER_EXTRA_SIZE);
    
    logger(LOG_DEBUG) << "USBBulkStreamer: Setting buffer size = " << bufSize << std::endl;
    
    return true;
  }

  bool putBuffer()
  {
    Lock<Mutex> _(_dataAccessMutex);

    if(_inUseBuffers.size() >= MAX_BUFFER_COUNT)
    {
      logger(LOG_WARNING) << "USBBulkStreamer: Dropping a frame because of slow forward pipeline." << std::endl;
      _inUseBuffers.pop_front();
    }

    auto f = _rawBuffers.get();

    RawDataFramePtr &raw = *f;

    if(!raw || raw->data.size() != usbBuffer.size())
    {
      raw = RawDataFramePtr(new RawDataFrame());
      raw->data.resize(usbBuffer.size());
    }

    memcpy(raw->data.data(), usbBuffer.data(), usbBuffer.size() - BULK_XFER_EXTRA_SIZE);

    raw->timestamp = _timer.getCurrentRealTime(); // in micro seconds
    
    _inUseBuffers.push_back(f);

    _dataAvailableCondition.notify_all();

    return true;
  }

  // timeout in milliseconds
  bool getBuffer(long timeout, RawDataFramePtr &raw)
  {
    Lock<Mutex> _(_dataAccessMutex);

    if(!_dataAvailableCondition.wait_for(_, std::chrono::milliseconds(timeout), [this] { return _inUseBuffers.size() > 0; }))
      return false;

    FrameBuffer<RawDataFrame> f = *_inUseBuffers.begin();

    _inUseBuffers.pop_front();

    if(!raw || raw->data.size() != (*f)->data.size())
    {
      raw = RawDataFramePtr(new RawDataFrame());
      raw->data.resize((*f)->data.size());
    }

    memcpy(raw->data.data(), (*f)->data.data(), raw->data.size());
    raw->timestamp = (*f)->timestamp;

    f.reset();
    return true;
  }

};

USBBulkStreamer::USBBulkStreamer(USBIOPtr &usbIO, DevicePtr device, uint8_t endpoint): Streamer(device)
{
  _usbBulkStreamerPrivate = Ptr<USBBulkStreamerPrivate> (new USBBulkStreamerPrivate(usbIO, device, endpoint));
}

USBBulkStreamer::~USBBulkStreamer()
{
}

bool USBBulkStreamer::_start()
{
  return _usbBulkStreamerPrivate->start();
}

bool USBBulkStreamer::_capture(RawDataFramePtr &p)
{
  if(_usbBulkStreamerPrivate->getBuffer(2000, p))
  {
    p->id = _currentID++;
    return true;
  }
  return false;
}

bool USBBulkStreamer::_stop()
{
  return _usbBulkStreamerPrivate->stop();
}

bool USBBulkStreamer::isInitialized()
{
  return true;
}

bool USBBulkStreamer::getSupportedVideoModes(Vector<VideoMode> &videoModes)
{
  videoModes.clear();
  return true; // No specific video modes
}

bool USBBulkStreamer::getCurrentVideoMode(VideoMode &videoMode)
{
  return false;
}

bool USBBulkStreamer::setVideoMode(const VideoMode &videoMode)
{
  return true;
}

bool USBBulkStreamer::setBufferSize(size_t bufferSize)
{
  return _usbBulkStreamerPrivate->setBufferSize(bufferSize);
}


}
