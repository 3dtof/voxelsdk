/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_PYLOGGER_OUTPUT_STREAM_H
#define VOXEL_PYLOGGER_OUTPUT_STREAM_H

#include "../Logger.h"

namespace swig
{

template <class Type>
inline PyObject *from_ptr(Type* val, int owner);

void unchainSwigPyObject(SwigPyObject *root, SwigPyObject *obj);

}

namespace Voxel
{

class PyLoggerOutputStream
{
  PyObject *func;
  PyObject *self;
  PyLoggerOutputStream& operator=(const PyLoggerOutputStream&); // Not allowed
public:
  PyLoggerOutputStream(const PyLoggerOutputStream& o) : func(o.func), self(o.self) {
    PyEval_InitThreads();
    Py_XINCREF(func);
    Py_XINCREF(self);
  }
  PyLoggerOutputStream(PyObject *self, PyObject *func) : func(func), self(self) {
    PyEval_InitThreads();
    Py_XINCREF(this->func);
    Py_XINCREF(this->self);
    assert(PyCallable_Check(this->func));
  }
  ~PyLoggerOutputStream() {
    Py_XDECREF(func);
    Py_XDECREF(self);
  }
  
  void operator()(const String &out) {
    
    if (!self || !func || Py_None == func)
      return;
    
    PyGILState_STATE d_gstate;
    
    d_gstate = PyGILState_Ensure();

    PyObject *args = Py_BuildValue("(s#)", out.data(), out.size());
  
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