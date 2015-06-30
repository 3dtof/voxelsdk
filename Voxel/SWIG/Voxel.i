/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 *
 */

%module Voxel

// NOTE: If you are modifying any of the included .i files, please touch 'Voxel.i' as well.
//       Otherwise the python bindings are NOT rebuilt.

%include "common_defs.i"

%{
#include "../Configuration.h"
#include "../TinyXML2.h"
#include "../Downloader.h"
#include "../UVC.h"
#include "../UVCXU.h"
#include "../ParameterDMLParser.h"
#include "../USBSystem.h"
#include "../CameraSystem.h"
#include "../FrameStream.h"
#include "../FrameGenerator.h"
#include "PyDepthCameraCallback.h"
#include "PyLoggerOutputStream.h"
#include "PyProgressFunction.h"
#include "../Logger.h"
#include "../Convolve2D.h"
%}

%include "VoxelExports.h"
#define __attribute__(x)

%include "../Common.h"
%include "../Device.h"
%include "../Point.h"
%include "../VideoMode.h"

%handle_reference(Voxel::FrameSize);
%apply Voxel::FrameSize &OUTPUT { Voxel::FrameSize &s };
%apply const Voxel::FrameSize &INPUT { const Voxel::FrameSize &s };

%handle_reference(Voxel::FrameRate);
%apply Voxel::FrameRate &OUTPUT { Voxel::FrameRate &r };
%apply const Voxel::FrameRate &INPUT { const Voxel::FrameRate &r };

%apply float &OUTPUT { float &fovHalfAngle };

%basic_reference_type(value);
%basic_reference_type(v);

%apply unsigned int &OUTPUT { unsigned int &bpp };
%apply unsigned int &INPUT { const unsigned int &bpp };


%include "../SerializedObject.h"
%include "Frame.i"

%include "../RegisterProgrammer.h"
%include "../Parameter.h"
%include "../FrameBuffer.h"
%include "../DataPacket.h"
%include "../FrameStream.h"
%include "../FrameGenerator.h"
%include "../Filter/FilterParameter.h"

namespace std 
{
%template(DeviceVector) vector<Voxel::DevicePtr>;
%template(StringVector) vector<string>;
%template(PointVector) vector<Voxel::Point>;
%template(StringMap) unordered_map<string, string>;
%template(IntStringMap) unordered_map<int, string>;
%template(ParamMap) unordered_map<string, Voxel::Ptr<Voxel::Parameter>>;
%template(FilterParamMap) unordered_map<string, Voxel::Ptr<Voxel::FilterParameter>>;
%template(VideoModeVector) vector<Voxel::SupportedVideoMode>;
%template(Uint16Vector) vector<uint16_t>;
%template(Int16Vector) vector<int16_t>;
%template(Uint8Vector) vector<uint8_t>;
%template(FloatVector) vector<float>;
%template(IntegerVector) vector<int>;
%template(UnsignedIntegerVector) vector<uint>;
%template(FrameVector) vector<Voxel::FramePtr>;

}

%handle_reference(std::vector<Voxel::SupportedVideoMode>);
%apply std::vector<Voxel::SupportedVideoMode> &OUTPUT { std::vector<Voxel::SupportedVideoMode> &supportedVideoModes };

%handle_reference(Voxel::VideoMode);
%apply Voxel::VideoMode &OUTPUT { Voxel::VideoMode &videoMode };

%handle_reference(Voxel::ConfigSet);
%handle_reference(Voxel::ConfigSet *);
%apply Voxel::ConfigSet *&OUTPUT { Voxel::ConfigSet *&configSet };

%handle_reference(Voxel::RegionOfInterest);
%apply Voxel::RegionOfInterest &OUTPUT { Voxel::RegionOfInterest &roi };
%apply const Voxel::RegionOfInterest &INPUT { const Voxel::RegionOfInterest &roi };

%handle_reference(std::vector<float>);
%apply std::vector<float> &OUTPUT { std::vector<float> &out };

%typemap(in,numinputs=0) PyObject *selfObject { $1 = self; }

%extend Voxel::Downloader {
  void setLogCallback(PyObject *callback, PyObject *selfObject)
  {
    $self->setLogCallback(Voxel::PyLoggerOutputStream(selfObject, callback));
  }
  
  void setProgressFunction(PyObject *callback, PyObject *selfObject)
  {
    $self->setProgressFunction(Voxel::PyProgressFunction(selfObject, callback));
  }
  
  bool download(const String &name)
  {
    bool b;
    Py_BEGIN_ALLOW_THREADS
    b = $self->download(name);
    Py_END_ALLOW_THREADS
    return b;
  }
}
%ignore Voxel::Downloader::download();

%extend Voxel::LoggerOutStream {
  void setOutputFunction(PyObject *callback, PyObject *selfObject)
  {
    $self->setOutputFunction(Voxel::PyLoggerOutputStream(selfObject, callback));
  }
}

%extend Voxel::Logger {
  IndexType addOutputStream(PyObject *callback, PyObject *selfObject)
  {
    return $self->addOutputStream(Voxel::PyLoggerOutputStream(selfObject, callback));
  }
}

%include "../Convolve2D.h"
%include "../Filter/Filter.h"
%include "../Filter/FilterFactory.h"
%include "../Filter/FilterSet.h"
%include "../Configuration.h"
%include "../Logger.h"
%include "../Downloader.h"
%include "../ParameterDMLParser.h"
%include "../PointCloudTransform.h"
%include "../Streamer.h"
%include "../Timer.h"
%include "../USBSystem.h"
%include "../UVC.h"
%include "../UVCXU.h"

%extend Voxel::DepthCamera {
  bool registerCallback(FrameType type, PyObject *callback, PyObject *selfObject)
  {
    return $self->registerCallback(type, Voxel::PyDepthCameraCallback(selfObject, callback));
  }
  
  bool stop()
  {
    bool b;
    Py_BEGIN_ALLOW_THREADS
    b = $self->stop();
    Py_END_ALLOW_THREADS
    return b;
  }
  
  void wait()
  {
    Py_BEGIN_ALLOW_THREADS
    $self->wait();
    Py_END_ALLOW_THREADS
  }
}
%ignore Voxel::DepthCamera::stop();
%ignore Voxel::DepthCamera::wait();

%include "../DepthCamera.h"
%include "../DepthCameraFactory.h"
%include "../DownloaderFactory.h"
%include "../CameraSystem.h"
%include "PyDepthCameraCallback.h"
%include "PyLoggerOutputStream.h"
%include "PyProgressFunction.h"

%template(seti) Voxel::DepthCamera::set<int>;
%template(setu) Voxel::DepthCamera::set<uint>;
%template(setf) Voxel::DepthCamera::set<float>;
%template(setb) Voxel::DepthCamera::set<bool>;
%template(geti) Voxel::DepthCamera::get<int>;
%template(getu) Voxel::DepthCamera::get<uint>;
%template(getf) Voxel::DepthCamera::get<float>;
%template(getb) Voxel::DepthCamera::get<bool>;

%template(getStreamParams) Voxel::DepthCamera::getStreamParam<Voxel::String>;
%template(getStreamParami) Voxel::DepthCamera::getStreamParam<int32_t>;
%template(getStreamParamu) Voxel::DepthCamera::getStreamParam<uint32_t>;
%template(getStreamParamf) Voxel::DepthCamera::getStreamParam<float>;

%template(getStreamParams) Voxel::FrameStreamReader::getStreamParam<Voxel::String>;
%template(getStreamParami) Voxel::FrameStreamReader::getStreamParam<int32_t>;
%template(getStreamParamu) Voxel::FrameStreamReader::getStreamParam<uint32_t>;
%template(getStreamParamf) Voxel::FrameStreamReader::getStreamParam<float>;

%template(seti) Voxel::Filter::set<int>;
%template(setu) Voxel::Filter::set<uint>;
%template(setf) Voxel::Filter::set<float>;
%template(setb) Voxel::Filter::set<bool>;
%template(geti) Voxel::Filter::get<int>;
%template(getu) Voxel::Filter::get<uint>;
%template(getf) Voxel::Filter::get<float>;
%template(getb) Voxel::Filter::get<bool>;

%template(XYZPointCloudFrame) Voxel::PointCloudFrameTemplate<Voxel::Point>;
%template(XYZIPointCloudFrame) Voxel::PointCloudFrameTemplate<Voxel::IntensityPoint>;
%template(ToF1608Frame) Voxel::ToFRawFrameTemplate<uint16_t, uint8_t>;
%template(ToF16IQFrame) Voxel::ToFRawIQFrameTemplate<int16_t>;
%template(RawFrameFilterSet) Voxel::FilterSet<Voxel::RawFrame>;
%template(DepthFrameFilterSet) Voxel::FilterSet<Voxel::DepthFrame>;
%template(XYZIPointCloudFrameFilterSet) Voxel::FilterSet<Voxel::XYZIPointCloudFrame>;
%template(RawFrameFilterSetIterator) Voxel::FilterSetIterator<Voxel::RawFrame>;
%template(DepthFrameFilterSetIterator) Voxel::FilterSetIterator<Voxel::DepthFrame>;
%template(XYZIPointCloudFrameFilterSetIterator) Voxel::FilterSetIterator<Voxel::XYZIPointCloudFrame>;