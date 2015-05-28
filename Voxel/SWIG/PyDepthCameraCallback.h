/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PYCALLBACK
#define VOXEL_PYCALLBACK

#include "../DepthCamera.h"

namespace swig
{

template <class Type>
inline PyObject *from_ptr(Type* val, int owner);

void unchainSwigPyObject(SwigPyObject *root, SwigPyObject *obj)
{
  if(obj == root)
    return;
  
  SwigPyObject *cur = (SwigPyObject *)root->next, *prev = root;
  
  while(cur)
  {
    if(obj == cur)
    {
      prev->next = cur->next;
      return;
    }
    prev = cur;
    cur = (SwigPyObject *)cur->next;
  }
}

}

namespace Voxel
{

class PyDepthCameraCallback
{
    PyObject *func;
    PyObject *self;
    PyDepthCameraCallback& operator=(const PyDepthCameraCallback&); // Not allowed
public:
    PyDepthCameraCallback(const PyDepthCameraCallback& o) : func(o.func), self(o.self) {
      PyEval_InitThreads();
      Py_XINCREF(func);
      Py_XINCREF(self);
    }
    PyDepthCameraCallback(PyObject *self, PyObject *func) : func(func), self(self) {
      PyEval_InitThreads();
      Py_XINCREF(this->func);
      Py_XINCREF(this->self);
      assert(PyCallable_Check(this->func));
    }
    ~PyDepthCameraCallback() {
      Py_XDECREF(func);
      Py_XDECREF(self);
    }
    
    void operator()(DepthCamera &camera, const Frame &frame, Voxel::DepthCamera::FrameType callBackType) {
      
      if (!self || !func || Py_None == func)
        return;
      
      PyGILState_STATE d_gstate;
      
      d_gstate = PyGILState_Ensure();

      PyObject *cam = SWIG_Python_NewPointerObj(self, SWIG_as_voidptr(new Voxel::shared_ptr<DepthCamera>(&camera, [](DepthCamera *) {})), 
                                                SWIGTYPE_p_Voxel__shared_ptrT_Voxel__DepthCamera_t, SWIG_BUILTIN_INIT); //SWIG_BUILTIN_INIT
      PyObject *frm = SWIG_Python_NewPointerObj(self, SWIG_as_voidptr(new Voxel::shared_ptr<Frame>((Frame *)&frame, [](Frame *) {})), 
                                                SWIGTYPE_p_Voxel__shared_ptrT_Voxel__Frame_t, SWIG_BUILTIN_INIT); //SWIG_BUILTIN_INIT

      PyObject *args = Py_BuildValue("(S,S,i)", cam, frm, callBackType);
    
      PyObject *result = PyObject_Call(func,args,0);
      
      if(PyErr_Occurred())
        PyErr_Print();
      
      Py_DECREF(args);
      swig::unchainSwigPyObject((SwigPyObject *)self, (SwigPyObject *)cam);
      Py_DECREF(cam);
      swig::unchainSwigPyObject((SwigPyObject *)self, (SwigPyObject *)frm);
      Py_DECREF(frm);
      Py_XDECREF(result);
      PyGILState_Release(d_gstate);
    }
};

}

#endif //VOXEL_PYCALLBACK