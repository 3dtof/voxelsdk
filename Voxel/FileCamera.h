/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FILECAMERA_H
#define VOXEL_FILECAMERA_H


#include <Device.h>
#include <Parameter.h>
#include <Device.h>
#include <Parameter.h>
#include <Frame.h>
#include "VideoMode.h"
#include <FrameBuffer.h>

#include <RegisterProgrammer.h>
#include <Streamer.h>

#include <PointCloudTransform.h>

#include <Filter/FilterSet.h>
#include "FrameStream.h"
#include "FrameGenerator.h"
#include "PointCloudFrameGenerator.h"
#include "Configuration.h"

namespace Voxel
{
  
/**
  * \ingroup CamSys
  * 
  * \brief This class inherits from DepthCamera and can be used to process recorded data. 
  * 
  * FileCamera is base class for processing files containing data captured from depth cameras. 
  * for individual depth camera types.
  */
class VOXEL_EXPORT FileCamera
{
   public:
      FileCamera(double len): _length(len) {}  // This is the constructor
      void setLength( double len );
      // double getLength( void );

      virtual ~FileCamera();
 
   private:
      double _length;
};

typedef Ptr<FileCamera> FileCameraPtr;
}

#endif // FILECAMERA_H
