/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FRAME_H
#define VOXEL_FRAME_H

#include <Common.h>
#include <Point.h>
#include "VideoMode.h"

#include <sstream>

namespace Voxel
{

class Frame
{
public:
  TimeStampType timestamp = 0; // Unix timestamp in micro-seconds
  int id = -1;
  
  inline operator String()
  {
    std::ostringstream s;
    s << id << "@" << timestamp;
    return s.str();
  }
  
  virtual ~Frame() {}
};

class DepthFrame: public Frame
{
public:
  Vector<float> depth; // depth frame row-wise. Unit: meters
  Vector<float> amplitude; // amplitude of each depth pixel normalized to value between 0 and 1
  SizeType size[2];
  
  virtual ~DepthFrame() {}
};

typedef Ptr<DepthFrame> DepthFramePtr;

class RawFrame: public Frame
{
public:
  virtual ~RawFrame() {}
};

typedef Ptr<RawFrame> RawFramePtr;

class ToFRawFrame: public RawFrame
{
public:
  virtual uint8_t *phase() = 0;
  virtual SizeType phaseWordWidth() = 0; // in bytes

  virtual uint8_t *amplitude() = 0;
  virtual SizeType amplitudeWordWidth() = 0; // in bytes

  virtual uint8_t *flags() = 0;
  virtual SizeType flagsWordWidth() = 0; // in bytes

  virtual uint8_t *ambient() = 0;
  virtual SizeType ambientWordWidth() = 0; // in bytes

  virtual uint16_t *histogram() = 0;
  virtual SizeType histogramSize() = 0; // number of elements in the histogram

  FrameSize size;
  
  virtual ~ToFRawFrame() {}
};

typedef Ptr<ToFRawFrame> ToFRawFramePtr;

template <typename PhaseByteType, typename AmbientByteType>
class ToFRawFrameTemplate: public ToFRawFrame
{
public:
  typedef PhaseByteType AmplitudeByteType;
  typedef AmbientByteType FlagsByteType;
  
  Vector<PhaseByteType> _phase;
  Vector<AmplitudeByteType> _amplitude;

  Vector<AmbientByteType> _ambient;
  Vector<FlagsByteType> _flags;

  Vector<uint16_t> _histogram;

  virtual uint8_t *ambient()
  {
    return (uint8_t *)_ambient.data();
  }
  virtual SizeType ambientWordWidth()
  {
    return sizeof(AmbientByteType);
  }

  virtual uint8_t *amplitude()
  {
    return (uint8_t *)_amplitude.data();
  }
  
  virtual SizeType amplitudeWordWidth()
  {
    return sizeof(AmplitudeByteType);
  }

  virtual uint8_t *phase()
  {
    return (uint8_t *)_phase.data();
  }
  
  virtual SizeType phaseWordWidth()
  {
    return sizeof(PhaseByteType);
  }

  virtual uint8_t *flags()
  {
    return (uint8_t *)_flags.data();
  }
  
  virtual SizeType flagsWordWidth()
  {
    return sizeof(FlagsByteType);
  }
  
  virtual uint16_t *histogram()
  {
    return _histogram.data();
  }
  
  virtual SizeType histogramSize()
  {
    return _histogram.size();
  }
  
  virtual ~ToFRawFrameTemplate() {}
};

class RawDataFrame: public RawFrame
{
public:
  Vector<ByteType> data;
  
  virtual ~RawDataFrame() {}
};

typedef Ptr<RawDataFrame> RawDataFramePtr;


class PointCloudFrame: public Frame
{
public:
  virtual SizeType size() = 0;
  virtual Point *operator [](IndexType index) = 0;
  
  virtual ~PointCloudFrame() {}
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
  
  virtual ~PointCloudFrameTemplate() {}
};

typedef PointCloudFrameTemplate<Point> XYZPointCloudFrame;
typedef PointCloudFrameTemplate<IntensityPoint> XYZIPointCloudFrame;

typedef Ptr<XYZPointCloudFrame> XYZPointCloudFramePtr;

}

#endif // VOXEL_POINT_H
