/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "UVCStreamer.h"
#include "Logger.h"


#include <errno.h>
#include <memory.h>
#include <assert.h>

#define MAX_BUFFER_COUNT 2

#ifdef LINUX
#include <linux/videodev2.h>
#include "UVCPrivateLinux.h"
#elif defined(WINDOWS)
#include "FrameBuffer.h"
#include "UVCPrivateWindows.h"

#include <dshow.h>
#include <comdef.h>


/// ISampleGrabber interface is not available in headers currently. Adding it directly here.
DEFINE_GUID(IID_ISampleGrabber, 0x6b652fff, 0x11fe, 0x4fce, 0x92, 0xad, 0x02, 0x66, 0xb5, 0xd7, 0xc7, 0x8f);

DEFINE_GUID(CLSID_NullRenderer, 0xc1f400a4, 0x3f08, 0x11d3, 0x9f, 0x0b, 0x00, 0x60, 0x08, 0x03, 0x9e, 0x37);
static const CLSID CLSID_SampleGrabber = { 0xC1F400A0, 0x3F08, 0x11d3, { 0x9F, 0x0B, 0x00, 0x60, 0x08, 0x03, 0x9E, 0x37 } };


interface ISampleGrabberCB : public IUnknown
{
  virtual HRESULT STDMETHODCALLTYPE SampleCB(
    double SampleTime,
    IMediaSample *pSample) = 0;

  virtual HRESULT STDMETHODCALLTYPE BufferCB(
    double SampleTime,
    BYTE *pBuffer,
    LONG BufferLen) = 0;

  virtual ~ISampleGrabberCB() {}
};

interface ISampleGrabber : public IUnknown
{
  virtual HRESULT STDMETHODCALLTYPE SetOneShot(
    BOOL OneShot) = 0;

  virtual HRESULT STDMETHODCALLTYPE SetMediaType(
    const AM_MEDIA_TYPE *pType) = 0;

  virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType(
    AM_MEDIA_TYPE *pType) = 0;

  virtual HRESULT STDMETHODCALLTYPE SetBufferSamples(
    BOOL BufferThem) = 0;

  virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer(
    LONG *pBufferSize,
    LONG *pBuffer) = 0;

  virtual HRESULT STDMETHODCALLTYPE GetCurrentSample(
    IMediaSample **ppSample) = 0;

  virtual HRESULT STDMETHODCALLTYPE SetCallback(
    ISampleGrabberCB *pCallback,
    LONG WhichMethodToCallback) = 0;

  virtual ~ISampleGrabber() {}
};

#endif

namespace Voxel
{

class UVCStreamer::UVCStreamerPrivate
#ifdef WINDOWS
  : public ISampleGrabberCB
#endif
{
  UVCStreamer &_uvcStreamer;
public:
#ifdef LINUX
  enum CaptureMode
  {
    CAPTURE_READ_WRITE,
    CAPTURE_MMAP,
    CAPTURE_USER_POINTER,
    CAPTURE_STREAMING // This is an intermediate which will be used to decide whether CAPTURE_MMAP or CAPTURE_USER_POINTER can be used
  } captureMode;

  Vector<UVCRawData> rawDataBuffers;

  size_t frameByteSize;

  void updateFrameByteSize(uint32_t width, uint32_t height, uint32_t bytesPerLine, uint32_t frameSize)
  {
    /* Buggy driver paranoia. */
    size_t min = width * 2;
    if (bytesPerLine < min)
      bytesPerLine = min;

    min = bytesPerLine * height;

    if (frameSize < min)
      frameByteSize = min;
    else
      frameByteSize = frameSize;
  }


#elif defined(WINDOWS)
  ICaptureGraphBuilder2 *captureGraphBuilder2 = 0;
  IGraphBuilder *filterGraph = 0;
  IMediaFilter *mediaFilter = 0;
  IMediaControl *mediaControl = 0;
  IMediaEventEx *mediaEventEx = 0;
  IBaseFilter *sampleGrabberFilter = 0;
  ISampleGrabber *sampleGrabber = 0;
  IBaseFilter *nullRendererFilter = 0;
  IAMDroppedFrames *droppedFrames = 0;
  IAMVideoProcAmp *videoProcAmp = 0;
#endif
  bool initialized = true;
  Ptr<UVC> uvc;

  UVCStreamerPrivate(UVCStreamer &uvcStreamer):
#ifdef WINDOWS
    _rawBuffers(2),
#endif
    _uvcStreamer(uvcStreamer)
  {
  }

  ~UVCStreamerPrivate()
  {
#if defined(WINDOWS)
    if (captureGraphBuilder2) captureGraphBuilder2->Release();
    if (filterGraph) filterGraph->Release();
    if (mediaFilter) mediaFilter->Release();
    if (mediaControl) mediaControl->Release();
    if (mediaEventEx) mediaEventEx->Release();
    if (sampleGrabber) sampleGrabber->Release();
    if (sampleGrabberFilter) sampleGrabberFilter->Release();
    if (nullRendererFilter) nullRendererFilter->Release();
    if (droppedFrames) droppedFrames->Release();
    if (videoProcAmp) videoProcAmp->Release();
#endif
  }

#ifdef WINDOWS
protected:
  TimeStampType _sampleStart = 0;
  Timer _timer;

  FrameBufferManager<RawDataFrame> _rawBuffers;

  List<FrameBuffer<RawDataFrame>> _inUseBuffers;

  Mutex _dataAccessMutex;
  ConditionVariable _dataAvailableCondition;

public:
  STDMETHODIMP_(ULONG) AddRef() { return 1; }
  STDMETHODIMP_(ULONG) Release() { return 2; }


  STDMETHODIMP QueryInterface(REFIID, void **ppvObject)
  {
    *ppvObject = static_cast<ISampleGrabberCB*>(this);
    return S_OK;
  }

  STDMETHODIMP SampleCB(double, IMediaSample *pSample)
  {
    return E_NOTIMPL;
  }

  STDMETHODIMP BufferCB(double timestamp, BYTE *buffer, long bufferLength)
  {
    if (!_uvcStreamer.isRunning())
      return S_OK;

    if (_sampleStart == 0)
    {
      _sampleStart = _timer.getCurrentRealTime() - (TimeStampType)(timestamp*1E6); // in micro seconds
    }

    logger(LOG_DEBUG) << "UVCStreamer: Got sample buffer at " << timestamp << std::endl;

    Lock<Mutex> _(_dataAccessMutex);

    if (_inUseBuffers.size() >= MAX_BUFFER_COUNT)
    {
      logger(LOG_WARNING) << "UVCStreamer: Dropping a frame because of slow forward pipeline." << std::endl;
      _inUseBuffers.pop_front();
    }

    auto f = _rawBuffers.get();

    RawDataFramePtr &raw = *f;

    if (!raw || raw->data.size() != bufferLength)
    {
      raw = RawDataFramePtr(new RawDataFrame());
      raw->data.resize(bufferLength);
    }

    memcpy(raw->data.data(), buffer, bufferLength);

    raw->timestamp = _sampleStart + (TimeStampType)(timestamp*1E6);

    _inUseBuffers.push_back(f);

    _dataAvailableCondition.notify_all();

    return S_OK;
  }

  // timeout in milliseconds
  bool getBuffer(long timeout, RawDataFramePtr &raw)
  {
    Lock<Mutex> _(_dataAccessMutex);

    if (!_dataAvailableCondition.wait_for(_, std::chrono::milliseconds(timeout), [this] { return _inUseBuffers.size() > 0; }))
      return false;

    FrameBuffer<RawDataFrame> f = *_inUseBuffers.begin();

    _inUseBuffers.pop_front();

    if (!raw || raw->data.size() != (*f)->data.size())
    {
      raw = RawDataFramePtr(new RawDataFrame());
      raw->data.resize((*f)->data.size());
    }

    memcpy(raw->data.data(), (*f)->data.data(), raw->data.size());
    raw->timestamp = (*f)->timestamp;

    f.reset();
    return true;
  }

  // The following two functions add and remove Directshow graph
  DWORD rotRegister = 0;
  HRESULT AddGraphToRot()
  {
    IMoniker *pMoniker;
    IRunningObjectTable *pROT;
    WCHAR wsz[128];
    HRESULT hr;

    if (!filterGraph || rotRegister)
      return E_POINTER;
    if (FAILED(hr = GetRunningObjectTable(0, &pROT)))
    {
      logger(LOG_WARNING) << "Could not get ROT. Error: " << _com_error(hr).ErrorMessage();
      return E_FAIL;
    }
    hr = StringCchPrintfW(wsz, NUMELMS(wsz), L"FilterGraph %08x pid %08x\0", (DWORD_PTR)filterGraph,
      GetCurrentProcessId());
    hr = CreateItemMoniker(L"!", wsz, &pMoniker);
    if (SUCCEEDED(hr)) 
    {
      // Use the ROTFLAGS_REGISTRATIONKEEPSALIVE to ensure a strong reference
      // to the object.  Using this flag will cause the object to remain
      // registered until it is explicitly revoked with the Revoke() method.
      //
      // Not using this flag means that if GraphEdit remotely connects
      // to this graph and then GraphEdit exits, this object registration
      // will be deleted, causing future attempts by GraphEdit to fail until
      // this application is restarted or until the graph is registered again.
      hr = pROT->Register(ROTFLAGS_REGISTRATIONKEEPSALIVE, filterGraph,
        pMoniker, &rotRegister);
    }
    else 
      logger(LOG_WARNING) << "Could not get item moniker. Error: " << _com_error(hr).ErrorMessage();
    pMoniker->Release();
    pROT->Release();
    return hr;
  }


  // Removes a filter graph from the Running Object Table
  void RemoveGraphFromRot()
  {
    if (!rotRegister)
      return;

    IRunningObjectTable *pROT;
    if (SUCCEEDED(GetRunningObjectTable(0, &pROT))) 
    {
      pROT->Revoke(rotRegister);
      pROT->Release();
      rotRegister = 0;
    }
  }

  // Release the format block for a media type.
  void FreeMediaType(AM_MEDIA_TYPE& mt)
  {
    if (mt.cbFormat != 0)
    {
      CoTaskMemFree((PVOID)mt.pbFormat);
      mt.cbFormat = 0;
      mt.pbFormat = NULL;
    }
    if (mt.pUnk != NULL)
    {
      // pUnk should not be used.
      mt.pUnk->Release();
      mt.pUnk = NULL;
    }
  }

  // Delete a media type structure that was allocated on the heap.
  void DeleteMediaType(AM_MEDIA_TYPE *pmt)
  {
    if (pmt != NULL)
    {
      FreeMediaType(*pmt);
      CoTaskMemFree(pmt);
    }
  }
#endif
};

bool UVCStreamer::isInitialized()
{
  return _uvcStreamerPrivate->uvc && _uvcStreamerPrivate->initialized;
}
  
UVCStreamer::UVCStreamer(Voxel::DevicePtr device) : Streamer(device)
{ 
  if (!_uvcInit() && _uvcStreamerPrivate)
  {
    _uvcStreamerPrivate->initialized = false;
    return;
  }
}

bool UVCStreamer::_uvcInit()
{
  if(_device->interfaceID() != Device::USB)
    return false;
  
  _uvcStreamerPrivate = Ptr<UVCStreamerPrivate>(new UVCStreamerPrivate(*this));
  _uvcStreamerPrivate->uvc = Ptr<UVC>(new UVC(_device));
  
#ifdef WINDOWS
  if (_uvcStreamerPrivate->uvc->isInitialized())
  {
    HRESULT hr;
    if ((hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC, IID_IGraphBuilder, (void **)&_uvcStreamerPrivate->filterGraph)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to create filter graph. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->QueryInterface(IID_IMediaControl, (LPVOID *)&_uvcStreamerPrivate->mediaControl)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to get interface for media control. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->QueryInterface(IID_IMediaEventEx, (LPVOID *)&_uvcStreamerPrivate->mediaEventEx)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to get interface for media filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->QueryInterface(IID_IMediaFilter, (LPVOID *)&_uvcStreamerPrivate->mediaFilter)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to get interface for media filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->AddFilter(_uvcStreamerPrivate->uvc->getUVCPrivate().getCaptureFilter(), L"UVC Capture")) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to add UVC capture filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    // Create the Sample Grabber filter.
    if ((hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (LPVOID *)&_uvcStreamerPrivate->sampleGrabberFilter)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to create sample grabber filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->AddFilter(_uvcStreamerPrivate->sampleGrabberFilter, L"Sample Grabber")) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to add sample grabber filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->sampleGrabberFilter->QueryInterface(IID_ISampleGrabber, (LPVOID *)&_uvcStreamerPrivate->sampleGrabber)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to get sample grabber interface. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    AM_MEDIA_TYPE mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));

    mt.majortype = MEDIATYPE_Video;
    mt.subtype = MEDIASUBTYPE_YUY2;
    mt.formattype = FORMAT_VideoInfo;

    if ((hr = _uvcStreamerPrivate->sampleGrabber->SetMediaType(&mt)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to set YUY2 as media subtype. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->sampleGrabber->SetBufferSamples(FALSE)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to configure sample grabber. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->sampleGrabber->SetOneShot(FALSE)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to configure sample grabber. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->sampleGrabber->SetCallback(&*_uvcStreamerPrivate, 1)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to configure sample grabber. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, IID_IBaseFilter, (LPVOID *)&_uvcStreamerPrivate->nullRendererFilter)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to create null renderer filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->filterGraph->AddFilter(_uvcStreamerPrivate->nullRendererFilter, L"Null Renderer")) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to add null renderer filter. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC, IID_ICaptureGraphBuilder2, (void **)&_uvcStreamerPrivate->captureGraphBuilder2)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to create capture graph builder. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->captureGraphBuilder2->SetFiltergraph(_uvcStreamerPrivate->filterGraph)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to set filter graph to capture graph builder. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }

    if ((hr = _uvcStreamerPrivate->captureGraphBuilder2->RenderStream(&PIN_CATEGORY_CAPTURE, &MEDIATYPE_Video,
      _uvcStreamerPrivate->uvc->getUVCPrivate().getCaptureFilter(), _uvcStreamerPrivate->sampleGrabberFilter, _uvcStreamerPrivate->nullRendererFilter)) != S_OK)
    {
      logger(LOG_ERROR) << "Failed to connect filters in the capture graph builder. Error: " << _com_error(hr).ErrorMessage();
      return _uvcStreamerPrivate->initialized = false;
    }
  }
#endif
  
  return _uvcStreamerPrivate->uvc->isInitialized();

}

bool UVCStreamer::getCurrentVideoMode(VideoMode &videoMode)
{
  if(!isInitialized())
    return false;
  
#ifdef LINUX
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current frame format" << std::endl;
    return false;
  }
  
  videoMode.frameSize.width = fmt.fmt.pix.width;
  videoMode.frameSize.height = fmt.fmt.pix.height;
  
  _uvcStreamerPrivate->updateFrameByteSize(fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_G_PARM, &parm) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current capture parameters" << std::endl;
    return false;
  }
  
  videoMode.frameRate.numerator = parm.parm.capture.timeperframe.denominator;
  videoMode.frameRate.denominator = parm.parm.capture.timeperframe.numerator;
#elif defined(WINDOWS)

#endif

  return true;
}

void UVCStreamer::_storeCurrentVideoMode(const VideoMode &videoMode)
{
  if (!_currentVideoMode)
    _currentVideoMode = Ptr<VideoMode>(new VideoMode());

  *_currentVideoMode = videoMode;
}

bool UVCStreamer::setVideoMode(const VideoMode &videoMode)
{
  if(!isInitialized() || isRunning())
    return false;
  
  logger(LOG_DEBUG) << "UVCStreamer: Setting video mode = " 
    << videoMode.frameSize.width << "x" << videoMode.frameSize.height 
    << "@" << videoMode.getFrameRate() << "fps" << std::endl;
  
#ifdef LINUX
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current frame format" << std::endl;
    return false;
  }
  
  fmt.fmt.pix.width = videoMode.frameSize.width;
  fmt.fmt.pix.height = videoMode.frameSize.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_S_FMT, &fmt) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not set current frame format" << std::endl;
    return false;
  }
  
  /// Get once more to set frame size
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_G_FMT, &fmt) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not set current frame format" << std::endl;
    return false;
  }
  
  _uvcStreamerPrivate->updateFrameByteSize(fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
  
  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_G_PARM, &parm) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current capture parameters" << std::endl;
    return false;
  }
  
  parm.parm.capture.timeperframe.denominator = videoMode.frameRate.numerator;
  parm.parm.capture.timeperframe.numerator = videoMode.frameRate.denominator;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_S_PARM, &parm) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not set current capture parameters" << std::endl;
    return false;
  }
#elif defined(WINDOWS)
  HRESULT hr;
  IAMStreamConfig *config;
  if((hr = _uvcStreamerPrivate->captureGraphBuilder2->FindInterface(&PIN_CATEGORY_CAPTURE, &MEDIATYPE_Video, 
    _uvcStreamerPrivate->uvc->getUVCPrivate().getCaptureFilter(), IID_IAMStreamConfig, (void**)&config)) != S_OK)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get stream configurator. Error: " << _com_error(hr).ErrorMessage();
    return false;
  }

  AM_MEDIA_TYPE *mtConfig = NULL;
  if ((hr = config->GetFormat(&mtConfig)) != S_OK)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current format. Error: " << _com_error(hr).ErrorMessage();
    config->Release();
    return false;
  }

  VIDEOINFOHEADER *videoHeader = (VIDEOINFOHEADER*)mtConfig->pbFormat;
  videoHeader->AvgTimePerFrame = videoMode.frameRate.denominator / videoMode.frameRate.numerator;
  videoHeader->bmiHeader.biWidth = videoMode.frameSize.width;
  videoHeader->bmiHeader.biHeight = videoMode.frameSize.height;

  if ((hr = config->SetFormat(mtConfig)) != S_OK)
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get current format. Error: " << _com_error(hr).ErrorMessage();
    config->Release();
    _uvcStreamerPrivate->DeleteMediaType(mtConfig);
    return false;
  }

  config->Release();
  _uvcStreamerPrivate->DeleteMediaType(mtConfig);

#endif
  _storeCurrentVideoMode(videoMode);
  return true;
}


bool UVCStreamer::_initForCapture()
{
  if(!_uvcInit())
    return false;
  
#ifdef LINUX
  ///// 1. Figure out which mode of capture to use
  struct v4l2_capability cap;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_QUERYCAP, &cap) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " is not a V4L2 device" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }
  
  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " is no video capture device" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }
  
  if(cap.capabilities & V4L2_CAP_READWRITE)
  {
    logger(LOG_INFO) << "UVCStreamer: " << _device->id() << " does not support read()/write() calls" << std::endl;
    _uvcStreamerPrivate->captureMode = UVCStreamerPrivate::CAPTURE_READ_WRITE;
  }
  else if(cap.capabilities & V4L2_CAP_STREAMING)
  {
    logger(LOG_INFO) << "UVCStreamer: " << _device->id() << " supports streaming modes" << std::endl;
    _uvcStreamerPrivate->captureMode = UVCStreamerPrivate::CAPTURE_STREAMING;
  }
#endif
  
  //// 2. Figure out frame size and frame rate
  if (_currentVideoMode && !setVideoMode(*_currentVideoMode))
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not set the current video mode" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }

  VideoMode currentVideoMode;
  
  if(!getCurrentVideoMode(currentVideoMode))
  {
    logger(LOG_ERROR) << "UVCStreamer: Could not get the current video mode" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }
  
  return _uvcStreamerPrivate->initialized = true;
}

bool UVCStreamer::_start()
{
  if(!_initForCapture())
    return false;
  
#ifdef LINUX
  if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_STREAMING)
  {
    struct v4l2_requestbuffers req;
    
    memset(&req, 0, sizeof(req));
    
    req.count = MAX_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
    {
      if(EINVAL == errno)
      {
        logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " does not support mmap" << std::endl;
        
        memset(&req, 0, sizeof(req));
        
        req.count = MAX_BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;
        
        if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
        {
          if(EINVAL == errno)
          {
            logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " does not support user pointer" << std::endl;
            return _uvcStreamerPrivate->initialized = false; // No usable capture mode available
          }
        }
        else
        {
          logger(LOG_INFO) << "UVCStreamer: " << _device->id() << " supports user pointer" << std::endl;
          _uvcStreamerPrivate->captureMode = UVCStreamerPrivate::CAPTURE_USER_POINTER;
          
          if(req.count < 2)
          {
            logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " insufficient buffers to capture" << std::endl;
            return _uvcStreamerPrivate->initialized = false;
          }
          
          _uvcStreamerPrivate->rawDataBuffers.resize(req.count);
        }
      }
      else
      {
        logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " VIDIC_REQBUFS failed" << std::endl;
        return _uvcStreamerPrivate->initialized = false;
      }
    }
    else
    {
      logger(LOG_INFO) << "UVCStreamer: " << _device->id() << " supports mmap" << std::endl;
      _uvcStreamerPrivate->captureMode = UVCStreamerPrivate::CAPTURE_MMAP;
      
      if(req.count < 2)
      {
        logger(LOG_ERROR) << "UVCStreamer: " << _device->id() << " insufficient buffers to capture" << std::endl;
        return _uvcStreamerPrivate->initialized = false;
      }
      
      _uvcStreamerPrivate->rawDataBuffers.resize(req.count);
    }
  }
  
  //// Initialize _uvcStreamerPrivate->rawDataBuffers
  if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP)
  {
    for(auto i = 0; i < _uvcStreamerPrivate->rawDataBuffers.size(); i++)
    {
      struct v4l2_buffer buf;
      
      memset(&buf, 0, sizeof(buf));
      
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      
      if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_QUERYBUF, &buf) == -1)
      {
        logger(LOG_ERROR) << "UVCStreamer: Could not prepare raw data buffer '" << i << "'" << std::endl;
        return _uvcStreamerPrivate->initialized = false;
      }
      
      _uvcStreamerPrivate->rawDataBuffers[i].size = buf.length;
      
      if(!_uvcStreamerPrivate->uvc->getUVCPrivate().mmap(buf.m.offset, _uvcStreamerPrivate->rawDataBuffers[i]))
      {
        logger(LOG_ERROR) << "UVCStreamer: Failed to get raw memory mapped buffer '" << i << "'" << std::endl;
        return _uvcStreamerPrivate->initialized = false;
      }
    }
  }
  else if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_USER_POINTER)
  {
    for(auto i = 0; i < _uvcStreamerPrivate->rawDataBuffers.size(); i++)
    {
      _uvcStreamerPrivate->rawDataBuffers[i].size = _uvcStreamerPrivate->frameByteSize;
      _uvcStreamerPrivate->rawDataBuffers[i].data = Ptr<ByteType>(new ByteType[_uvcStreamerPrivate->frameByteSize], 
                                              [](ByteType *d){ delete []d; });
    }
  }
  
  /// Enqueue _uvcStreamerPrivate->rawDataBuffers and start streaming
  if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP || _uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_USER_POINTER)
  {
    for(auto i = 0; i < _uvcStreamerPrivate->rawDataBuffers.size(); i++)
    {
      struct v4l2_buffer buf;
      
      memset(&buf, 0, sizeof(buf));
      
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = (_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
      buf.index  = i;
      
      if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_USER_POINTER)
      {
        buf.m.userptr = (unsigned long)&*_uvcStreamerPrivate->rawDataBuffers[i].data;
        buf.length = _uvcStreamerPrivate->rawDataBuffers[i].size;
      }
      
      if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_QBUF, &buf) == -1)
      {
        logger(LOG_ERROR) << "UVCStreamer: Could not queue raw data buffer '" << i << "'" << std::endl;
        return _uvcStreamerPrivate->initialized = false;
      }
    }
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_STREAMON, &type) == -1)
    {
      logger(LOG_ERROR) << "UVCStreamer: Failed to start capture stream" << std::endl;
      return _uvcStreamerPrivate->initialized = false;
    }
  }
#elif defined(WINDOWS)
  HRESULT hr;
  if ((hr = _uvcStreamerPrivate->mediaControl->Run()) < 0)
  {
    logger(LOG_ERROR) << "Failed to start the capture filter. Error: " << _com_error(hr).ErrorMessage();
    return false;
  }

  OAFilterState state;
  if ((hr == _uvcStreamerPrivate->mediaControl->GetState(INFINITE, &state)) != S_OK)
  {
    logger(LOG_ERROR) << "Failed to start the capture filter. Error: " << _com_error(hr).ErrorMessage();
    return false;
  }

  if (state != State_Running)
  {
    logger(LOG_ERROR) << "Filter is not running. Current state: " << state << std::endl;
    return false;
  }

  if ((hr = _uvcStreamerPrivate->AddGraphToRot()) != S_OK) // Allow for graph debugging via graphedit
  {
    logger(LOG_WARNING) << "Could not add filter graph to ROT" << std::endl;
  }
#endif

  return true;
}

bool UVCStreamer::_stop()
{
  if(!isInitialized())
    return false;

#ifdef LINUX  
  /// Stop streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_STREAMOFF, &type) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Failed to stop capture stream" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }
  
  /// Remove MMAPs if any
  _uvcStreamerPrivate->uvc->getUVCPrivate().clearMMap();
  
  /// Remove requestbuffers
  struct v4l2_requestbuffers req;
  
  memset(&req, 0, sizeof(req));
  
  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = (_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
  
  if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_REQBUFS, &req) == -1)
  {
    logger(LOG_ERROR) << "UVCStreamer: Failed to remove buffers" << std::endl;
    return _uvcStreamerPrivate->initialized = false;
  }
  
  /// Remove buffers
  _uvcStreamerPrivate->rawDataBuffers.clear();
#elif defined(WINDOWS)
  _uvcStreamerPrivate->mediaControl->Stop();
  long evCode;
  _uvcStreamerPrivate->mediaEventEx->WaitForCompletion(500, &evCode);
  _uvcStreamerPrivate->RemoveGraphFromRot(); // Remove graph for debugging via graphedit
#endif
  
  return true;
}

bool UVCStreamer::_capture(RawDataFramePtr &p)
{
  TimeStampType waitTime = 2000;//ms
  
#ifdef LINUX
  bool timedOut;
  if(!isInitialized() || !_uvcStreamerPrivate->uvc->getUVCPrivate().isReadReady(waitTime, timedOut))
  {
    if(timedOut)
      logger(LOG_ERROR) << "No data available. Waited for " << waitTime << " ms" << std::endl;
    
    return false;
  }
  
  if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_READ_WRITE)
  {
    if(!p || p->data.size() != _uvcStreamerPrivate->frameByteSize)
    {
      p = RawDataFramePtr(new RawDataFrame());
      p->data.resize(_uvcStreamerPrivate->frameByteSize);
      logger(LOG_DEBUG) << "UVCStreamer: Frame provided is not of appropriate size. Recreating a new frame." << std::endl;
    }
    
    bool ret = _uvcStreamerPrivate->uvc->read(p->data.data(), _uvcStreamerPrivate->frameByteSize);
    
    if(ret)
    {
      p->id = _currentID++;
      p->timestamp = _time.getCurrentRealTime();
      return true;
    }
    return false;
  }
  else if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP || _uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_USER_POINTER)
  {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = (_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_MMAP)?V4L2_MEMORY_MMAP:V4L2_MEMORY_USERPTR;
    
    if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_DQBUF, &buf) == -1)
    {
      switch(errno)
      {
        case EIO:
          /* Could ignore EIO, see spec. */
          /* fall through */
          
        default:
          logger(LOG_ERROR) << "UVCStreamer: Failed to dequeue a raw frame buffer" << std::endl;
          return false;
      }
    }
    
    if(_uvcStreamerPrivate->captureMode == UVCStreamerPrivate::CAPTURE_USER_POINTER)
    {
      buf.index = _uvcStreamerPrivate->rawDataBuffers.size();
      for(auto i = 0; i < _uvcStreamerPrivate->rawDataBuffers.size(); i++)
        if(&*_uvcStreamerPrivate->rawDataBuffers[i].data == (ByteType *)buf.m.userptr && _uvcStreamerPrivate->rawDataBuffers[i].size == buf.bytesused)
        {
          buf.index = i;
          break;
        }
    }
    
    assert(buf.index < _uvcStreamerPrivate->rawDataBuffers.size());
    
    if(buf.bytesused < _uvcStreamerPrivate->frameByteSize)
    {
      logger(LOG_ERROR) << "Incomplete frame data. Skipping it. Expected bytes = " 
      << _uvcStreamerPrivate->frameByteSize << ", got bytes = " << buf.bytesused << std::endl;
      
      if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_QBUF, &buf) == -1)
      {
        logger(LOG_ERROR) << "UVCStreamer: Failed to enqueue back the raw frame buffer" << std::endl;
        return false;
      }
      
      return false;
    }
    
    if(!p || p->data.size() != buf.bytesused)
    {
      p = RawDataFramePtr(new RawDataFrame());
      p->data.resize(buf.bytesused);
      logger(LOG_DEBUG) << "UVCStreamer: Frame provided is not of appropriate size. Recreating a new frame." << std::endl;
    }
    
    p->timestamp = _time.convertToRealTime(buf.timestamp.tv_sec*1000000L + buf.timestamp.tv_usec);
    memcpy(p->data.data(), &*_uvcStreamerPrivate->rawDataBuffers[buf.index].data, buf.bytesused);
    
    if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_QBUF, &buf) == -1)
    {
      logger(LOG_ERROR) << "UVCStreamer: Failed to enqueue back the raw frame buffer" << std::endl;
      return false;
    }
    
    p->id = _currentID++;
    
    return true;
  }
#elif defined(WINDOWS)
  if (_uvcStreamerPrivate->getBuffer(waitTime, p))
  {
    p->id = _currentID++;
    return true;
  }
  return false;
#endif
}

bool UVCStreamer::getSupportedVideoModes(Vector<VideoMode> &videoModes)
{
  if(!isInitialized())
    return false;
  
  videoModes.clear();
  
#ifdef LINUX
  int frameSizeIndex = 0;
  int frameRateIndex = 0;
  
  uint32_t frameWidth, frameHeight;
  
  
  struct v4l2_frmsizeenum frameSizeEnum;
  struct v4l2_frmivalenum frameIntervalEnum;
  
  while(1)
  {
    memset(&frameSizeEnum, 0, sizeof(frameSizeEnum));
    
    frameSizeEnum.index = frameSizeIndex++;
    frameSizeEnum.pixel_format = V4L2_PIX_FMT_YUYV;//V4L2_PIX_FMT_UYVY;//V4L2_PIX_FMT_YUYV;
    
    if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_ENUM_FRAMESIZES, &frameSizeEnum) == -1)
    {
      if(errno == EINVAL)
        break;
      else
        return false;
    }
    
    if(frameSizeEnum.type != V4L2_FRMSIZE_TYPE_DISCRETE)
    {
      logger(LOG_WARNING) << "Frame types other than discrete, are not supported currently." << std::endl;
      continue;
    }
    
    frameWidth = frameSizeEnum.discrete.width;
    frameHeight = frameSizeEnum.discrete.height;
    
    frameRateIndex = 0;
    while(1)
    {
      memset(&frameIntervalEnum, 0, sizeof(frameIntervalEnum));
      
      frameIntervalEnum.index = frameRateIndex++;
      frameIntervalEnum.pixel_format = V4L2_PIX_FMT_YUYV;
      frameIntervalEnum.width = frameWidth;
      frameIntervalEnum.height = frameHeight;
      
      if(_uvcStreamerPrivate->uvc->getUVCPrivate().xioctl(VIDIOC_ENUM_FRAMEINTERVALS, &frameIntervalEnum) == -1)
      {
        if(errno == EINVAL)
          break;
        else
          return false;
      }
      
      if(frameIntervalEnum.type != V4L2_FRMIVAL_TYPE_DISCRETE)
      {
        logger(LOG_WARNING) << "Frame interval types other than discrete, are not supported currently." << std::endl;
        continue;
      }
      
      VideoMode vm;
      vm.frameSize.width = frameWidth;
      vm.frameSize.height = frameHeight;
      vm.frameRate.numerator = frameIntervalEnum.discrete.denominator;
      vm.frameRate.denominator = frameIntervalEnum.discrete.numerator;
      
      videoModes.push_back(vm);
    }
  }
#elif defined(WINDOWS)
#endif
  return true;
}

UVCStreamer::~UVCStreamer()
{
  if(isInitialized() && isRunning())
    stop(); // NOTE This cannot be called in the base class desctructor as UVCStreamer would already be destroyed by then
}

  
}