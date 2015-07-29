/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

%include "macros.i"

%make_ptr(Device);
%make_ptr(USBDevice);

%make_ptr(Downloader);
%make_ptr(USBDownloader);

%make_ptr(Filter);
%make_ptr(FilterFactory);

%make_ptr(DepthCamera);
%make_ptr(DepthCameraFactory);
%make_ptr(DepthCameraLibrary);

%make_ptr(RegisterProgrammer);
%make_ptr(Streamer);

%make_ptr(Frame);
%make_ptr(RawFrame);
%make_ptr(RawDataFrame);
%make_ptr(ToFRawFrame);
%make_ptr(ToFRawIQFrame);
%make_ptr(DepthFrame);
%make_ptr(PointCloudFrame);
%make_ptr(PointCloudFrameTemplate<Voxel::Point>);
%make_ptr(PointCloudFrameTemplate<Voxel::IntensityPoint>);
%make_ptr(ToFRawFrameTemplate<uint16_t, uint8_t>);
%make_ptr(ToFRawIQFrameTemplate<int16_t>);
%make_ptr(Parameter);
%make_ptr(ParameterTemplate<bool>);
%make_ptr(ParameterTemplate<int>);
%make_ptr(ParameterTemplate<uint>);
%make_ptr(ParameterTemplate<float>);
%make_ptr(EnumParameterTemplate<bool>);
%make_ptr(EnumParameterTemplate<int>);
%make_ptr(BoolParameter);
%make_ptr(StrobeBoolParameter);
%make_ptr(EnumParameter);
%make_ptr(RangeParameterTemplate<int>);
%make_ptr(RangeParameterTemplate<uint>);
%make_ptr(RangeParameterTemplate<float>);
%make_ptr(IntegerParameter);
%make_ptr(UnsignedIntegerParameter);
%make_ptr(FloatParameter);

%make_ptr(FilterParameter);
%make_ptr(FilterParameterTemplate<bool>);
%make_ptr(FilterParameterTemplate<int>);
%make_ptr(FilterParameterTemplate<uint>);
%make_ptr(FilterParameterTemplate<float>);
%make_ptr(FilterParameterEnumTemplate<bool>);
%make_ptr(FilterParameterEnumTemplate<int>);
%make_ptr(BoolFilterParameter);
%make_ptr(FilterParameterRangeTemplate<int>);
%make_ptr(FilterParameterRangeTemplate<uint>);
%make_ptr(FilterParameterRangeTemplate<float>);

%make_ptr(FrameStreamReader);
%make_ptr(FrameStreamWriter);
%make_ptr(SerializedObject);
%make_ptr(FrameGenerator);
%make_ptr(DepthFrameGenerator);

%feature("python:slot", "tp_str", functype="reprfunc") *::__str__;
%feature("autodoc", "3");

