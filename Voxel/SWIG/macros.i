/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

%include "windows.i"

// TODO: This needs to be set based on whether the architecture is 32-bit or 64-bit.
#define SWIGWORDSIZE64 

%include "typemaps.i"
%include "std_string.i"
%include "std_vector.i"
%include "stdint.i"
%include "unordered_map_py.i"

#define SWIG_SHARED_PTR_NAMESPACE Voxel
%include "boost_shared_ptr.i"
#define Ptr shared_ptr

typedef unsigned int uint;

%define %define_swig_type(Type...)
%traits_swigtype(Type);
%fragment(SWIG_Traits_frag(Type));
%enddef

// Define reference handling for custom types. Also, make reference as an output argument
%define %handle_reference(Type...)
%typemap(in,numinputs=0) Type &OUTPUT ($*1_ltype temp)  { $1 = &temp; }

%typemap(in) const Type &INPUT { swig::asptr<Type>($input, &$1); };

%define_swig_type(Type)

%typemap(argout, fragment="t_output_helper") Type &OUTPUT { 
     // append
     PyObject* o = swig::from(*$1); 
     $result = t_output_helper( $result, o ); 
}

%typemap(argout) const Type &INPUT ""
%enddef

%define %make_ptr(Type...)
//namespace std { %template(Type ## SharedPtr) shared_ptr<Voxel::##Type>; %shared_ptr(Voxel::##Type); }
//namespace Voxel { %template(Type ## Ptr) Ptr<Voxel::##Type>; }
%shared_ptr(Voxel::##Type);
%enddef

%define %basic_reference_type(VarName...)
%apply float &OUTPUT { float &VarName };
%apply float &INPUT { const float &VarName };
%apply int &OUTPUT { int &VarName };
%apply int &INPUT { const int &VarName };
%apply unsigned int &OUTPUT { unsigned int &VarName };
%apply unsigned int &INPUT { const unsigned int &VarName };
%apply bool &OUTPUT { bool &VarName };
%apply bool &INPUT { const bool &VarName };
%enddef
