/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

%feature("python:slot", "mp_subscript", functype="binaryfunc") Voxel::PointCloudFrame::__getitem__;
%feature("python:bf_getbuffer", functype="getbufferproc") Voxel::PointCloudFrameTemplate<Voxel::IntensityPoint> "getXYZIPointCloudFrameBuffer";
%feature("python:bf_releasebuffer", functype="releasebufferproc") Voxel::PointCloudFrameTemplate<Voxel::IntensityPoint> "releaseXYZIPointCloudFrameBuffer";

%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<float> "getVectorBuffer<float, 0, 'f'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<float> "releaseBuffer";
%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<uint16_t> "getVectorBuffer<uint16_t, 4, 'H'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<uint16_t> "releaseBuffer";
%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<int16_t> "getVectorBuffer<int16_t, 4, 'h'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<int16_t> "releaseBuffer";
%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<uint8_t> "getVectorBuffer<uint8_t, 3, 'B'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<uint8_t> "releaseBuffer";
%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<int> "getVectorBuffer<int, 2, 'i'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<int> "releaseBuffer";
%feature("python:bf_getbuffer", functype="getbufferproc") std::vector<bool> "getVectorBuffer<bool, 1, '?'>";
%feature("python:bf_releasebuffer", functype="releasebufferproc") std::vector<bool> "releaseBuffer";

%inline %{
#undef Py_TPFLAGS_DEFAULT 
#define Py_TPFLAGS_DEFAULT (Py_TPFLAGS_DEFAULT_EXTERNAL|Py_TPFLAGS_HAVE_NEWBUFFER) 
%}

%include "../Frame.h"

%extend Voxel::Frame {
  Voxel::String __str__() {
    return $self->operator Voxel::String();
  }
}

%extend Voxel::PointCloudFrame {
  Point *__getitem__(IndexType index) {
    return $self->operator[](index);
  }
  
}

%inline %{

static swig_type_info *supportedVectorTypes[] = {
 SWIGTYPE_p_std__vectorT_float_std__allocatorT_float_t_t,
 SWIGTYPE_p_std__vectorT_bool_std__allocatorT_bool_t_t,
 SWIGTYPE_p_std__vectorT_int_std__allocatorT_int_t_t,
 SWIGTYPE_p_std__vectorT_unsigned_char_std__allocatorT_unsigned_char_t_t,
 SWIGTYPE_p_std__vectorT_unsigned_short_std__allocatorT_unsigned_short_t_t
};

namespace Voxel
{
struct BufferView
{
  void *buf;
  Py_ssize_t len;
  Py_ssize_t itemsize;  /* This is Py_ssize_t so it can be
                            pointed to by strides in simple case.*/
  int readonly;
  int ndim;
  char format[2];
  int refCount;
};

struct Buffer1DView: public BufferView
{
  Py_ssize_t shape;
  Py_ssize_t strides;
};

struct Buffer2DView: public BufferView
{
  Py_ssize_t shape[2];
  Py_ssize_t strides[2];
};

static std::unordered_map<PyObject *, Buffer1DView> buffer1DViewMap;
static std::unordered_map<PyObject *, Buffer2DView> buffer2DViewMap;

}

template <typename T, const int swig_type_index, const char type>
static int getVectorBuffer(PyObject *pyobj, Py_buffer *view, int flags) { 
  void *argp; 
  int res; 
  
  int own;
  res = SWIG_ConvertPtrAndOwn(pyobj, &argp, supportedVectorTypes[swig_type_index], 0, &own);
  
  if (!SWIG_IsOK(res)) {
    SWIG_Error(SWIG_ArgError(res), "in method 'getBuffer', argument 1 of type 'std::vector<T> *'"); 
    return 0;
  }
  
  std::vector<T> &v = *reinterpret_cast<std::vector<T>*>(argp);
  
  view->obj = pyobj;
  view->internal = NULL;

  auto i = Voxel::buffer1DViewMap.find(pyobj);

  if(i != Voxel::buffer1DViewMap.end())
  {
    Voxel::Buffer1DView &b = i->second;
    view->buf = b.buf; 
    view->len = b.len;
    view->readonly = b.readonly; 
    view->format = b.format;
    view->ndim = b.ndim; 
    view->shape = &b.shape;
    view->strides = &b.strides;
    view->suboffsets = NULL;
    view->itemsize = b.itemsize;
    b.refCount++;
  }
  else
  {
    Voxel::Buffer1DView &b = Voxel::buffer1DViewMap[pyobj];

    view->buf = b.buf = v.data(); 
    view->len = b.len = sizeof(T)*v.size();
    view->readonly = b.readonly = 0; 
    b.format[0] = type; b.format[1] = '\0';
    view->format = b.format;
    view->ndim = b.ndim = 1; 
    b.shape = v.size();
    b.strides = sizeof(T);
    view->shape = &b.shape;
    view->strides = &b.strides;
    view->suboffsets = NULL;
    view->itemsize = b.itemsize = sizeof(T);
    b.refCount = 1;
  }

  Py_INCREF(pyobj);

  //std::cout << "Address = 0x" << std::hex << (uint)v.data() << ", len = " << std::dec << view->len << std::hex << ", flags = " << flags
  //  << ", format = 0x" << (uint)view->format << ", strides = 0x" << (uint)view->strides << ", shape = 0x" << (uint)view->shape << std::dec << std::endl;
  return 0;
}

static void releaseBuffer(PyObject *pyobj, Py_buffer *view)
{
  //std::cout << "Release buffer at address = 0x" << std::hex << (uint)view->buf << ", len = " << std::dec << view->len << std::hex
  //  << ", format = 0x" << (uint)view->format << ", strides = 0x" << (uint)view->strides << ", shape = 0x" << (uint)view->shape << std::dec << std::endl;

  //std::cout << "stride = " << *view->strides << ", shape = " << *view->shape << std::endl;

  auto i = Voxel::buffer1DViewMap.find(pyobj);

  if(i == Voxel::buffer1DViewMap.end())
    return;

  i->second.refCount--;

  if(!i->second.refCount)
  {
    //std::cout << "Cleaning up buffer at address = 0x" << std::hex << (uint)view->buf << std::dec << std::endl;
    Voxel::buffer1DViewMap.erase(i);
  }
}

static int getXYZIPointCloudFrameBuffer(PyObject *pyobj, Py_buffer *view, int flags) { 
  void *argp; 
  int res; 
  
  int own;

  res = SWIG_ConvertPtrAndOwn(pyobj, &argp, SWIGTYPE_p_Voxel__shared_ptrT_Voxel__PointCloudFrame_t, 0, &own); 
  
  if (!SWIG_IsOK(res)) {
    SWIG_Error(SWIG_ArgError(res), "in method '" "getXYZIPointCloudFrameBuffer" "', argument " "1"" of type '" "Voxel::shared_ptr<Voxel::PointCloudFrame> *""'"); 
    return 0;
  }
  
  Voxel::PointCloudFrame *frame = &**reinterpret_cast<Voxel::shared_ptr<Voxel::PointCloudFrame>*>(argp);
  
  view->obj = pyobj;
  view->internal = NULL;

  auto i = Voxel::buffer2DViewMap.find(pyobj);

  if(i != Voxel::buffer2DViewMap.end())
  {
    Voxel::Buffer2DView &b = i->second;
    view->buf = b.buf; 
    view->len = b.len;
    view->readonly = b.readonly; 
    view->format = b.format; 
    view->ndim = b.ndim; 
    view->shape = b.shape;
    view->strides = b.strides;
    view->suboffsets = NULL;
    view->itemsize = b.itemsize;
    b.refCount++;
  }
  else
  {
    Voxel::Buffer2DView &b = Voxel::buffer2DViewMap[pyobj];
    view->buf = b.buf = (*frame)[0]; 
    view->len = b.len = sizeof(Voxel::IntensityPoint)*frame->size();
    view->readonly = b.readonly = 1; 
    b.format[0] = 'f'; b.format[1] = '\0';
    view->format = b.format; 
    view->ndim = b.ndim = 2; 
    b.shape[0] = frame->size(); b.shape[1] = 4;
    b.strides[0] = 4*sizeof(float); b.strides[1] = 1*sizeof(float); 
    view->shape = b.shape;
    view->strides = b.strides;
    view->suboffsets = NULL;
    view->itemsize = b.itemsize = sizeof(float);
    b.refCount = 1;
  }

  Py_INCREF(pyobj);

  return 0;
}

static void releaseXYZIPointCloudFrameBuffer(PyObject *pyobj, Py_buffer *view)
{
  auto i = Voxel::buffer2DViewMap.find(pyobj);

  if(i == Voxel::buffer2DViewMap.end())
    return;

  i->second.refCount--;

  if(!i->second.refCount)
  {
    Voxel::buffer2DViewMap.erase(i);
  }
}

%}