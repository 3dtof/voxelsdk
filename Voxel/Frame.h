/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FRAME_H
#define VOXEL_FRAME_H

#include "Common.h"
#include "Point.h"

namespace Voxel
{

class Frame
{
public:
  int timestamp = 0; // Unix timestamp in milliseconds
  int id = -1;
};

class DepthFrame: public Frame
{
public:
  Vector<float> depth; // depth frame row-wise. Unit: meters
  Vector<float> amplitude; // amplitude of each depth pixel normalized to value between 0 and 1
  SizeType size[2];
};

typedef Ptr<DepthFrame> DepthFramePtr;

class RawFrame: public Frame
{
};

class ToFRawFrame: public RawFrame
{
public:
  virtual void *phase() = 0;
  virtual SizeType phaseWordWidth() = 0; // in bytes

  virtual void *amplitude() = 0;
  virtual SizeType amplitudeWordWidth() = 0; // in bytes

  virtual void *flags() = 0;
  virtual SizeType flagsWordWidth() = 0; // in bytes

  virtual void *ambient() = 0;
  virtual SizeType ambientWordWidth() = 0; // in bytes

  virtual float *histogram() = 0;
  virtual SizeType histogramSize() = 0; // number of elements in the histogram

  SizeType size[2];
};

template <typename PhaseWidth, typename AmbientWidth>
class ToFRawFrameTemplate: public ToFRawFrame
{
protected:
  typedef PhaseWidth AmplitudeWidth;
  typedef AmbientWidth FlagsWidth;
  
  Vector<PhaseWidth> _phase;
  Vector<AmplitudeWidth> _amplitude;

  Vector<AmbientWidth> _ambient;
  Vector<FlagsWidth> _flags;

  Vector<float> _histogram;

  virtual void *ambient()
  {
    return _ambient.data();
  }
  virtual SizeType ambientWordWidth()
  {
    return sizeof(AmbientWidth);
  }

  virtual void *amplitude()
  {
    return _amplitude.data();
  }
  
  virtual SizeType amplitudeWordWidth()
  {
    return sizeof(AmplitudeWidth);
  }

  virtual void *phase()
  {
    return _phase.data();
  }
  
  virtual SizeType phaseWordWidth()
  {
    return sizeof(PhaseWidth);
  }

  virtual void *flags()
  {
    return _flags.data();
  }
  
  virtual SizeType flagsWordWidth()
  {
    return sizeof(FlagsWidth);
  }
  
  virtual float *histogram()
  {
    return _histogram.data();
  }
  
  virtual SizeType histogramSize()
  {
    return _histogram.size();
  }
};

class RawDataFrame: public Frame
{
public:
  Vector<ByteType> data;
};


class PointCloudFrame: public Frame
{
public:
  virtual SizeType size() = 0;
  virtual Point *operator [](IndexType index) = 0;
};

typedef Ptr<PointCloudFrame> PointCloudFramePtr;

template <typename PointType>
class PointCloudFrameTemplate: public PointCloudFrame
{
public:
  Vector<PointType> points;
  
  virtual SizeType size()
  {
    return points.size();
  }
  
  virtual Point* operator[] (IndexType index)
  {
    if(index < points.size() && index >= 0)
      return &points[index];
    else
      return 0;
  }
};

typedef PointCloudFrameTemplate<Point> XYZPointCloudFrame;
typedef PointCloudFrameTemplate<IntensityPoint> XYZIPointCloudFrame;

}

#endif // VOXEL_POINT_H
