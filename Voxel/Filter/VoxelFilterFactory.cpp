/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "VoxelFilterFactory.h"

#include <Filter/IIRFilter.h>
#include <Filter/MedianFilter.h>
#include <Filter/TemporalMedianFilter.h>
#include <Filter/SmoothFilter.h>
#include <Filter/BilateralFilter.h>

namespace Voxel
{

VoxelFilterFactory::VoxelFilterFactory(): FilterFactory("Voxel")
{
  _addSupportedFilters({
    FilterDescription("IIRFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new IIRFilter()); }),
    FilterDescription("MedianFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new MedianFilter()); }),
    FilterDescription("TemporalMedianFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new TemporalMedianFilter()); }),
    FilterDescription("SmoothFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new SmoothFilter()); }),
    FilterDescription("BilateralFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new BilateralFilter()); }),
  });
}
 
}