/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FILTER_H
#define VOXEL_FILTER_H


#include "Frame.h"

namespace Voxel
{
  
class DepthFrameFilter
{
public:
  virtual DepthFramePtr filter(DepthFramePtr in) = 0;
};

class PointCloudFrameFilter
{
public:
  virtual PointCloudFramePtr filter(PointCloudFramePtr in) = 0;
};


}


#endif //VOXEL_FILTER_H
