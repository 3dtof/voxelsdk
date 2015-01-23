/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */


#include "VoxelFilterFactory.h"

#include <Filter/IIRFilter.h>

namespace Voxel
{

VoxelFilterFactory::VoxelFilterFactory(): FilterFactory("Voxel")
{
  _addSupportedFilters({
    FilterDescription("IIRFilter", 
                      (1 << DepthCamera::FRAME_RAW_FRAME_PROCESSED) | 
                      (1 << DepthCamera::FRAME_DEPTH_FRAME),
                      []() -> FilterPtr { return FilterPtr(new IIRFilter()); })
  });
}
 
}