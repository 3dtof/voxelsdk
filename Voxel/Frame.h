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
#include "SerializedObject.h"

#include <sstream>


namespace Voxel
{
  
/**
 * \defgroup Frm Frame related classes
 * @{
 */

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
  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const = 0;
  virtual Ptr<Frame> newFrame() const = 0;
  
  virtual bool isSameType(const Frame &other) const = 0;
  virtual bool isSameSize(const Frame &other) const = 0;
  
  virtual bool serialize(SerializedObject &object) const = 0;
  
  virtual bool deserialize(SerializedObject &object) = 0;
  
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
    Ptr<Frame> f(new DepthFrame());
    return copyTo(f);
  }

  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const
  {
    if(!other || !isSameType(*other))
      other = Ptr<Frame>(new DepthFrame());

    DepthFrame *d = dynamic_cast<DepthFrame *>(other.get());

    d->id = id;
    d->timestamp = timestamp;
    d->depth = depth;
    d->amplitude = amplitude;
    d->size = size;
    return other;
  }
  
  virtual bool serialize(SerializedObject &object) const
  {
    size_t s = sizeof(id) + sizeof(timestamp) + depth.size()*sizeof(float)*2 + sizeof(size_t)*2;
    
    object.resize(s);
    
    object.put((const char *)&id, sizeof(id));
    object.put((const char *)&timestamp, sizeof(timestamp));
    
    object.put((const char *)&size.width, sizeof(size.width));
    object.put((const char *)&size.height, sizeof(size.height));
    
    object.put((const char *)depth.data(), sizeof(float)*depth.size());
    object.put((const char *)amplitude.data(), sizeof(float)*amplitude.size());
    return true;
  }
  
  virtual bool deserialize(SerializedObject &object)
  {
    if(!object.get((char *)&id, sizeof(id)) ||
      !object.get((char *)&timestamp, sizeof(timestamp)) ||
      !object.get((char *)&size.width, sizeof(size.width)) ||
      !object.get((char *)&size.height, sizeof(size.height)))
      return false;
    
    depth.resize(size.width*size.height);
    amplitude.resize(size.width*size.height);
    
    if(!object.get((char *)depth.data(), sizeof(float)*depth.size()) ||
    !object.get((char *)amplitude.data(), sizeof(float)*amplitude.size()))
      return false;
      
    return true;
  }
  
  virtual bool isSameType(const Frame &other) const
  {
    const DepthFrame *f = dynamic_cast<const DepthFrame *>(&other);
    return f;
  }
  
  virtual bool isSameSize(const Frame &other) const
  {
    const DepthFrame *f = dynamic_cast<const DepthFrame *>(&other);
    return (f && size == f->size);
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    DepthFrame *d = new DepthFrame();
    d->depth.resize(depth.size());
    d->amplitude.resize(amplitude.size());
    d->size = size;
    return FramePtr(d);
  }
  
  static Ptr<DepthFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<DepthFrame>(ptr);
  }
  
  virtual ~DepthFrame() {}
};

typedef Ptr<DepthFrame> DepthFramePtr;

class VOXEL_EXPORT RawFrame: public Frame
{
public:
  static Ptr<RawFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<RawFrame>(ptr);
  }
  
  virtual ~RawFrame() {}
};

typedef Ptr<RawFrame> RawFramePtr;

enum ToFFrameType {
  ToF_PHASE_AMPLITUDE,
  ToF_I_Q,
  ToF_QUAD
};

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
  
  static Ptr<ToFRawFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<ToFRawFrame>(ptr);
  }
  
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
    Ptr<Frame> f(new ToFRawFrameTemplate<PhaseByteType, AmbientByteType>());
    return copyTo(f);
  }

  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const
  {
    if(!other || !isSameType(*other))
      other = Ptr<Frame>(new ToFRawFrameTemplate<PhaseByteType, AmbientByteType>());

    auto *t = dynamic_cast<ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *>(other.get());

    t->id = id;
    t->timestamp = timestamp;
    t->_phase = _phase;
    t->_amplitude = _amplitude;
    t->_ambient = _ambient;
    t->_flags = _flags;
    t->_histogram = _histogram;
    t->size = size;
    return other;
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
  
  virtual bool isSameType(const Frame &other) const
  {
    const ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *f = dynamic_cast<const ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *>(&other);
    return f;
  }
  
  static Ptr<ToFRawFrameTemplate<PhaseByteType, AmbientByteType>> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<ToFRawFrameTemplate<PhaseByteType, AmbientByteType>>(ptr);
  }
  
  virtual bool isSameSize(const Frame &other) const
  {
    const ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *f = dynamic_cast<const ToFRawFrameTemplate<PhaseByteType, AmbientByteType> *>(&other);
    return (f && size == f->size);
  }
  
  virtual bool serialize(SerializedObject &object) const
  {
    size_t s = sizeof(id) + sizeof(timestamp) + 
      _phase.size()*sizeof(PhaseByteType) + 
      _amplitude.size()*sizeof(AmplitudeByteType) + 
      _ambient.size()*sizeof(AmbientByteType) + 
      _flags.size()*sizeof(FlagsByteType) + 
      _histogram.size()*sizeof(uint16_t) + 
      sizeof(size_t)*2;
    
    object.resize(s);
    
    object.put((const char *)&id, sizeof(id));
    object.put((const char *)&timestamp, sizeof(timestamp));
    
    size_t histogramSize = _histogram.size();
    object.put((const char *)&size.width, sizeof(size.width));
    object.put((const char *)&size.height, sizeof(size.height));
    object.put((const char *)&histogramSize, sizeof(histogramSize));
    
    object.put((const char *)_phase.data(), sizeof(PhaseByteType)*_phase.size());
    object.put((const char *)_amplitude.data(), sizeof(AmplitudeByteType)*_amplitude.size());
    object.put((const char *)_ambient.data(), sizeof(AmbientByteType)*_ambient.size());
    object.put((const char *)_flags.data(), sizeof(FlagsByteType)*_flags.size());
    
    if(histogramSize)
      object.put((const char *)_histogram.data(), sizeof(uint16_t)*_histogram.size());
    return true;
  }
  
  virtual bool deserialize(SerializedObject &object)
  {
    object.get((char *)&id, sizeof(id));
    object.get((char *)&timestamp, sizeof(timestamp));
    
    size_t histogramSize;
    object.get((char *)&size.width, sizeof(size.width));
    object.get((char *)&size.height, sizeof(size.height));
    object.get((char *)&histogramSize, sizeof(histogramSize));
    
    _phase.resize(size.width*size.height);
    _amplitude.resize(size.width*size.height);
    _ambient.resize(size.width*size.height);
    _flags.resize(size.width*size.height);
    
    object.get((char *)_phase.data(), sizeof(PhaseByteType)*_phase.size());
    object.get((char *)_amplitude.data(), sizeof(AmplitudeByteType)*_amplitude.size());
    object.get((char *)_ambient.data(), sizeof(AmbientByteType)*_ambient.size());
    object.get((char *)_flags.data(), sizeof(FlagsByteType)*_flags.size());
    
    if(histogramSize)
    {
      _histogram.resize(histogramSize);
      object.get((char *)_histogram.data(), sizeof(uint16_t)*_histogram.size());
    }
    else
      _histogram.clear();
    
    return true;
  }
  
  virtual ~ToFRawFrameTemplate() {}
};

class VOXEL_EXPORT ToFRawIQFrame: public RawFrame
{
public:
  virtual const uint8_t *i() const = 0;
  virtual uint8_t *i() = 0;
  virtual const uint8_t *q() const = 0;
  virtual uint8_t *q() = 0;
  virtual SizeType wordWidth() const = 0; // in bytes
  
  FrameSize size;
  
  static Ptr<ToFRawIQFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<ToFRawIQFrame>(ptr);
  }
  
  virtual ~ToFRawIQFrame() {}
};

typedef Ptr<ToFRawIQFrame> ToFRawIQFramePtr;

template <typename ByteType>
class ToFRawIQFrameTemplate : public ToFRawIQFrame
{
public:
  Vector<ByteType> _i;
  Vector<ByteType> _q;
  
  virtual const uint8_t *i() const
  {
    return (const uint8_t *)_i.data();
  }
  
  virtual uint8_t *i()
  {
    return (uint8_t *)_i.data();
  }
  
  virtual const uint8_t *q() const
  {
    return (const uint8_t *)_q.data();
  }
  
  virtual uint8_t *q()
  {
    return (uint8_t *)_q.data();
  }
  
  virtual SizeType wordWidth() const
  {
    return sizeof(ByteType);
  }

  virtual Ptr<Frame> copy() const
  {
    Ptr<Frame> f(new ToFRawIQFrameTemplate<ByteType>());
    return copyTo(f);
  }

  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const
  {
    if(!other || !isSameType(*other))
      other = Ptr<Frame>(new ToFRawIQFrameTemplate<ByteType>());

    auto *t = dynamic_cast<ToFRawIQFrameTemplate<ByteType> *>(other.get());
    t->id = id;
    t->timestamp = timestamp;
    t->_i = _i;
    t->_q = _q;
    t->size = size;
    return other;
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    ToFRawIQFrameTemplate<ByteType> *t = new ToFRawIQFrameTemplate<ByteType>();
    t->_i.resize(_i.size());
    t->_q.resize(_q.size());
    t->size = size;
    return FramePtr(t);
  }
  
  virtual bool isSameType(const Frame &other) const
  {
    const ToFRawIQFrameTemplate<ByteType> *f = dynamic_cast<const ToFRawIQFrameTemplate<ByteType> *>(&other);
    return f;
  }
  
  static Ptr<ToFRawIQFrameTemplate<ByteType>> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<ToFRawIQFrameTemplate<ByteType>>(ptr);
  }
  
  virtual bool isSameSize(const Frame &other) const
  {
    const ToFRawIQFrameTemplate<ByteType> *f = dynamic_cast<const ToFRawIQFrameTemplate<ByteType> *>(&other);
    return (f && size == f->size);
  }
  
  virtual bool serialize(SerializedObject &object) const
  {
    size_t s = sizeof(id) + sizeof(timestamp) + 
    _i.size()*sizeof(ByteType) +
    _q.size()*sizeof(ByteType) +
    sizeof(uint32_t)*2;
    
    object.resize(s);
    
    object.put((const char *)&id, sizeof(id));
    object.put((const char *)&timestamp, sizeof(timestamp));
    
    uint32_t x;
    x = sizeof(ByteType);
    object.put((const char *)&x, sizeof(x));
    
    x = size.width;
    object.put((const char *)&x, sizeof(x));
    x = size.height;
    object.put((const char *)&x, sizeof(x));
    
    object.put((const char *)_i.data(), sizeof(ByteType)*_i.size());
    object.put((const char *)_q.data(), sizeof(ByteType)*_q.size());
    return true;
  }
  
  virtual bool deserialize(SerializedObject &object)
  {
    object.get((char *)&id, sizeof(id));
    object.get((char *)&timestamp, sizeof(timestamp));
    
    uint32_t x;
    object.get((char *)&x, sizeof(x));
    
    if(x != sizeof(ByteType))
      return false;
    
    object.get((char *)&x, sizeof(x));
    size.width = x;
    object.get((char *)&x, sizeof(x));
    size.height = x;
    
    _i.resize(size.width*size.height);
    _q.resize(size.width*size.height);
    
    object.get((char *)_i.data(), sizeof(ByteType)*_i.size());
    object.get((char *)_q.data(), sizeof(ByteType)*_q.size());
    
    return true;
  }
  
  virtual ~ToFRawIQFrameTemplate() {}
};

class VOXEL_EXPORT RawDataFrame : public RawFrame
{
public:
  Vector<ByteType> data;

  virtual Ptr<Frame> copy() const
  {
    Ptr<Frame> f(new RawDataFrame());
    return copyTo(f);
  }

  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const
  {
    if(!other || !isSameType(*other))
      other = Ptr<Frame>(new RawDataFrame());

    auto *r = dynamic_cast<RawDataFrame *>(other.get());
    r->id = id;
    r->timestamp = timestamp;
    r->data = data;
    return other;
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    RawDataFrame *r = new RawDataFrame();
    r->data.resize(data.size());
    return FramePtr(r);
  }
  
  virtual bool isSameType(const Frame &other) const
  {
    const RawDataFrame *f = dynamic_cast<const RawDataFrame *>(&other);
    return f;
  }
  
  virtual bool isSameSize(const Frame &other) const
  {
    const RawDataFrame *f = dynamic_cast<const RawDataFrame *>(&other);
    return f && data.size() == f->data.size();
  }
  
  static Ptr<RawDataFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<RawDataFrame>(ptr);
  }
  
  virtual bool serialize(SerializedObject &object) const
  {
    size_t s = sizeof(id) + sizeof(timestamp) + data.size()*sizeof(ByteType) + sizeof(size_t);
    
    object.resize(s);
    
    object.put((const char *)&id, sizeof(id));
    object.put((const char *)&timestamp, sizeof(timestamp));
    
    s = data.size();
    object.put((const char *)&s, sizeof(s));
    
    object.put((const char *)data.data(), sizeof(ByteType)*data.size());
    return true;
  }
  
  virtual bool deserialize(SerializedObject &object)
  {
    size_t s;
    if(!object.get((char *)&id, sizeof(id)) ||
    !object.get((char *)&timestamp, sizeof(timestamp)) ||
    !object.get((char *)&s, sizeof(s)))
      return false;
    
    data.resize(s);
    
    return object.get((char *)data.data(), sizeof(ByteType)*data.size());
  }
  
  virtual ~RawDataFrame() {}
};

typedef Ptr<RawDataFrame> RawDataFramePtr;


class VOXEL_EXPORT PointCloudFrame : public Frame
{
public:
  virtual SizeType size() const = 0;
  virtual Point *operator [](IndexType index) = 0;
  
  static Ptr<PointCloudFrame> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<PointCloudFrame>(ptr);
  }
  
  virtual ~PointCloudFrame() {}
};

typedef Ptr<PointCloudFrame> PointCloudFramePtr;

template <typename PointType>
class PointCloudFrameTemplate : public PointCloudFrame
{
public:
  Vector<PointType> points;
  
  virtual SizeType size() const
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
    Ptr<Frame> f(new PointCloudFrameTemplate<PointType>());
    return copyTo(f);
  }

  virtual Ptr<Frame> copyTo(Ptr<Frame> &other) const
  {
    if(!other || !isSameType(*other))
      other = Ptr<Frame>(new PointCloudFrameTemplate<PointType>());

    auto *p = dynamic_cast<PointCloudFrameTemplate<PointType> *>(other.get());
    p->id = id;
    p->timestamp = timestamp;
    p->points = points;
    return other;
  }
  
  virtual Ptr<Frame> newFrame() const
  {
    PointCloudFrameTemplate<PointType> *p = new PointCloudFrameTemplate<PointType>();
    p->points.resize(points.size());
    return FramePtr(p);
  }
  
  virtual bool isSameType(const Frame &other) const
  {
    const PointCloudFrameTemplate<PointType> *f = dynamic_cast<const PointCloudFrameTemplate<PointType> *>(&other);
    return f;
  }
  
  virtual bool isSameSize(const Frame &other) const
  {
    const PointCloudFrameTemplate<PointType> *f = dynamic_cast<const PointCloudFrameTemplate<PointType> *>(&other);
    return f && f->size() == size();
  }
  
  virtual bool serialize(SerializedObject &object) const
  {
    size_t s = sizeof(id) + sizeof(timestamp) + points.size()*sizeof(PointType) + sizeof(size_t);
    
    object.resize(s);
    
    object.put((const char *)&id, sizeof(id));
    object.put((const char *)&timestamp, sizeof(timestamp));
    
    s = points.size();
    object.put((const char *)&s, sizeof(s));
    
    object.put((const char *)points.data(), sizeof(PointType)*points.size());
    return true;
  }
  
  virtual bool deserialize(SerializedObject &object)
  {
    object.get((char *)&id, sizeof(id));
    object.get((char *)&timestamp, sizeof(timestamp));
    
    size_t s;
    object.get((char *)&s, sizeof(s));
    
    points.resize(s);
    
    object.get((char *)points.data(), sizeof(ByteType)*points.size());
    return true;
  }
  
  static Ptr<PointCloudFrameTemplate<PointType>> typeCast(FramePtr ptr)
  {
    return std::dynamic_pointer_cast<PointCloudFrameTemplate<PointType>>(ptr);
  }
  
  virtual ~PointCloudFrameTemplate() {}
};

typedef PointCloudFrameTemplate<Point> XYZPointCloudFrame;
typedef PointCloudFrameTemplate<IntensityPoint> XYZIPointCloudFrame;

typedef Ptr<XYZPointCloudFrame> XYZPointCloudFramePtr;

/**
 * @}
 */

}

#endif // VOXEL_POINT_H
