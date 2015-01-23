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

class VOXEL_EXPORT Frame
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
  
  virtual Ptr<Frame> copy() const = 0;
  virtual Ptr<Frame> newFrame() const = 0;
  
  virtual ~Frame() {}
};

typedef Ptr<Frame> FramePtr;

class VOXEL_EXPORT DepthFrame: public Frame
{
public:
  Vector<float> depth; // depth frame row-wise. Unit: meters
  Vector<float> amplitude; // amplitude of each depth pixel normalized to value between 0 and 1
  FrameSize size;
  
  virtual Ptr<Frame> copy() const
  {
    DepthFrame *d = new DepthFrame();
    d->id = id;
    d->timestamp = timestamp;
    d->depth = depth;
    d->amplitude = amplitude;
    d->size = size;
    return FramePtr(d);
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    DepthFrame *d = new DepthFrame();
    d->depth.resize(depth.size());
    d->amplitude.resize(amplitude.size());
    d->size = size;
    return FramePtr(d);
  }
  
  virtual ~DepthFrame() {}
};

typedef Ptr<DepthFrame> DepthFramePtr;

class VOXEL_EXPORT RawFrame : public Frame
{
public:
  virtual ~RawFrame() {}
};

typedef Ptr<RawFrame> RawFramePtr;

class VOXEL_EXPORT ToFRawFrame : public RawFrame
{
public:
  virtual const uint8_t *phase() const = 0;
  virtual uint8_t *phase() = 0;
  virtual SizeType phaseWordWidth() const = 0; // in bytes

  virtual const uint8_t *amplitude() const = 0;
  virtual uint8_t *amplitude() = 0;
  virtual SizeType amplitudeWordWidth() const = 0; // in bytes

  virtual const uint8_t *flags() const = 0;
  virtual uint8_t *flags() = 0;
  virtual SizeType flagsWordWidth() const = 0; // in bytes

  virtual const uint8_t *ambient() const = 0;
  virtual uint8_t *ambient() = 0;
  virtual SizeType ambientWordWidth() const = 0; // in bytes

  virtual const uint16_t *histogram() const = 0;
  virtual uint16_t *histogram() = 0;
  virtual SizeType histogramSize() const = 0; // number of elements in the histogram

  FrameSize size;
  
  virtual ~ToFRawFrame() {}
};

typedef Ptr<ToFRawFrame> ToFRawFramePtr;

template <typename PhaseByteType, typename AmbientByteType>
class ToFRawFrameTemplate : public ToFRawFrame
{
public:
  typedef PhaseByteType AmplitudeByteType;
  typedef AmbientByteType FlagsByteType;
  
  Vector<PhaseByteType> _phase;
  Vector<AmplitudeByteType> _amplitude;

  Vector<AmbientByteType> _ambient;
  Vector<FlagsByteType> _flags;

  Vector<uint16_t> _histogram;

  virtual const uint8_t *ambient() const
  {
    return (const uint8_t *)_ambient.data();
  }
  
  virtual uint8_t *ambient()
  {
    return (uint8_t *)_ambient.data();
  }
  
  virtual SizeType ambientWordWidth() const
  {
    return sizeof(AmbientByteType);
  }

  virtual const uint8_t *amplitude() const
  {
    return (const uint8_t *)_amplitude.data();
  }
  
  virtual uint8_t *amplitude()
  {
    return (uint8_t *)_amplitude.data();
  }
  
  virtual SizeType amplitudeWordWidth() const
  {
    return sizeof(AmplitudeByteType);
  }

  virtual const uint8_t *phase() const
  {
    return (const uint8_t *)_phase.data();
  }
  
  virtual uint8_t *phase()
  {
    return (uint8_t *)_phase.data();
  }
  
  virtual SizeType phaseWordWidth() const
  {
    return sizeof(PhaseByteType);
  }

  virtual const uint8_t *flags() const
  {
    return (const uint8_t *)_flags.data();
  }
  
  virtual uint8_t *flags()
  {
    return (uint8_t *)_flags.data();
  }
  
  virtual SizeType flagsWordWidth() const
  {
    return sizeof(FlagsByteType);
  }
  
  virtual const uint16_t *histogram() const
  {
    return _histogram.data();
  }
  
  virtual uint16_t *histogram()
  {
    return _histogram.data();
  }
  
  virtual SizeType histogramSize() const
  {
    return _histogram.size();
  }
  
  virtual Ptr<Frame> copy() const
  {
    ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *t = new ToFRawFrameTemplate<PhaseByteType, AmbientByteType>();
    t->id = id;
    t->timestamp = timestamp;
    t->_phase = _phase;
    t->_amplitude = _amplitude;
    t->_ambient = _ambient;
    t->_flags = _flags;
    t->_histogram = _histogram;
    t->size = size;
    return FramePtr(t);
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *t = new ToFRawFrameTemplate<PhaseByteType, AmbientByteType>();
    t->_phase.resize(_phase.size());
    t->_amplitude.resize(_amplitude.size());
    t->_ambient.resize(_ambient.size());
    t->_flags.resize(_flags.size());
    t->_histogram.resize(_histogram.size());
    t->size = size;
    return FramePtr(t);
  }
  
  virtual ~ToFRawFrameTemplate() {}
};

class VOXEL_EXPORT RawDataFrame : public RawFrame
{
public:
  Vector<ByteType> data;
  
  virtual Ptr<Frame> copy() const
  {
    RawDataFrame *r = new RawDataFrame();
    r->id = id;
    r->timestamp = timestamp;
    r->data = data;
    return FramePtr(r);
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    RawDataFrame *r = new RawDataFrame();
    r->data.resize(data.size());
    return FramePtr(r);
  }
  
  virtual ~RawDataFrame() {}
};

typedef Ptr<RawDataFrame> RawDataFramePtr;


class VOXEL_EXPORT PointCloudFrame : public Frame
{
public:
  virtual SizeType size() = 0;
  virtual Point *operator [](IndexType index) = 0;
  
  virtual ~PointCloudFrame() {}
};

typedef Ptr<PointCloudFrame> PointCloudFramePtr;

template <typename PointType>
class PointCloudFrameTemplate : public PointCloudFrame
{
public:
  Vector<PointType> points;
  
  virtual SizeType size()
  {
    return points.size();
  }
  
  virtual Point *operator[] (IndexType index)
  {
    if(index < points.size() && index >= 0)
      return &points[index];
    else
      return 0;
  }
  
  virtual Ptr<Frame> copy() const
  {
    PointCloudFrameTemplate<PointType> *p = new PointCloudFrameTemplate<PointType>();
    p->id = id;
    p->timestamp = timestamp;
    p->points = points;
    return FramePtr(p);
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    PointCloudFrameTemplate<PointType> *p = new PointCloudFrameTemplate<PointType>();
    p->points.resize(points.size());
    return FramePtr(p);
  }
  
  virtual ~PointCloudFrameTemplate() {}
};

typedef PointCloudFrameTemplate<Point> XYZPointCloudFrame;
typedef PointCloudFrameTemplate<IntensityPoint> XYZIPointCloudFrame;

typedef Ptr<XYZPointCloudFrame> XYZPointCloudFramePtr;

}

#endif // VOXEL_POINT_H
