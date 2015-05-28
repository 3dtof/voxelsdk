/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_VOXEL_FILTER_FACTORY_H
#define VOXEL_VOXEL_FILTER_FACTORY_H

#include <Filter/FilterFactory.h>

namespace Voxel
{

/**
 * \addtogroup Flt
 * @{
 */  
class VOXEL_NO_EXPORT VoxelFilterFactory: public FilterFactory
{
public:
  VoxelFilterFactory();
  
  virtual ~VoxelFilterFactory() {}
};

/**
 * @}
 */

}

#endif // VOXEL_VOXEL_FILTER_FACTORY_H