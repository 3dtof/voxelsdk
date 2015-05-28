/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PYPROGRESS_FUNCTION_H
#define VOXEL_PYPROGRESS_FUNCTION_H

#include "../Logger.h"

namespace swig
{

template <class Type>
inline PyObject *from_ptr(Type* val, int owner);

void unchainSwigPyObject(SwigPyObject *root, SwigPyObject *obj);

}

namespace Voxel
{

class PyProgressFunction
{
  PyObject *func;
  PyObject *self;
  PyProgressFunction& operator=(const PyProgressFunction&); // Not allowed
public:
  PyProgressFunction(const PyProgressFunction& o) : func(o.func), self(o.self) {
    PyEval_InitThreads();
    Py_XINCREF(func);
    Py_XINCREF(self);
  }
  PyProgressFunction(PyObject *self, PyObject *func) : func(func), self(self) {
    PyEval_InitThreads();
    Py_XINCREF(this->func);
    Py_XINCREF(this->self);
    assert(PyCallable_Check(this->func));
  }
  ~PyProgressFunction() {
    Py_XDECREF(func);
    Py_XDECREF(self);
  }
  
  void operator()(float progress) {
    
    if (!self || !func || Py_None == func)
      return;
    
    PyGILState_STATE d_gstate;
    
    d_gstate = PyGILState_Ensure();

    PyObject *args = Py_BuildValue("(f)", progress);
  
    PyObject *result = PyObject_Call(func,args,0);
    
    if(PyErr_Occurred())
      PyErr_Print();
    
    Py_DECREF(args);
    Py_XDECREF(result);
    PyGILState_Release(d_gstate);
  }
};

}

#endif //VOXEL_PYCALLBACK