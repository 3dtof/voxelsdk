/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_FRAME_GENERATOR_H
#define VOXEL_FRAME_GENERATOR_H

#include <Frame.h>
#include <Common.h>

#include <Serializable.h>

namespace Voxel
{
  
class FrameStreamWriter;
  
class VOXEL_EXPORT FrameGenerator
{
protected:
  GeneratorIDType _id;
  int _frameType; // Generated frame type. Value as in Voxel::DepthCamera::FrameType
  Ptr<FrameStreamWriter> _frameStreamWriter;
  
  StringKeySerializableMap _frameGeneratorParameters; // Primarily for the purpose of serializing/deserializing these parameters
  
  uint8_t _majorVersion, _minorVersion; // Used for storing/reading configuration
  
  virtual bool _onReadConfiguration() = 0; // On read configuration
  virtual bool _onWriteConfiguration() = 0; // On write configuration
  
  inline bool _getParam(const String &name, SerializablePtr &param) const;
  
  template <typename T>
  inline bool _set(const String &name, const T &value); // Only for internal use
  
public:
  FrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion): _id(id), _frameType(frameType),
  _majorVersion(majorVersion), _minorVersion(minorVersion) 
  {
    _frameGeneratorParameters["version"] = 
        SerializablePtr(new SerializableUnsignedInt((majorVersion << 8) + minorVersion));
  }
  
  inline const GeneratorIDType &id() const { return _id; }
  
  inline void setFrameStreamWriter(Ptr<FrameStreamWriter> &writer) { _frameStreamWriter = writer; }
  inline void removeFrameStreamWriter() { _frameStreamWriter = nullptr; }
  
  virtual bool readConfiguration(SerializedObject &object); // Read configuration from serialized data object
  virtual bool writeConfiguration(); // Write configuration to FrameStreamWriter
  
  virtual bool generate(const FramePtr &in, FramePtr &out) = 0;
  
  template <typename T>
  inline bool get(const String &name, T &value) const;
  
  virtual ~FrameGenerator() {}
};

typedef Ptr<FrameGenerator> FrameGeneratorPtr;
inline bool FrameGenerator::_getParam(const String &name, SerializablePtr &param) const
{
  auto x = _frameGeneratorParameters.find(name);
  
  if(x != _frameGeneratorParameters.end())
  {
    param = x->second;
    return true;
  }
  
  return false;
}

template <typename T>
inline bool FrameGenerator::_set(const String &name, const T &value)
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  BasicSerializable<T> *p = dynamic_cast<BasicSerializable<T> *>(param.get());
  
  if(p)
  {
    p->value = value;
    return true;
  }
  return false;
}

template <>
inline bool FrameGenerator::_set<String>(const String &name, const String &value)
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  SerializableString *p = dynamic_cast<SerializableString *>(param.get());
  
  if(p)
  {
    *p = value;
    return true;
  }
  return false;
}

template <typename T>
inline bool FrameGenerator::get(const String &name, T &value) const
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  const BasicSerializable<T> *p = dynamic_cast<BasicSerializable<T> *>(param.get());
  
  if(p)
  {
    value = p->value;
    return true;
  }
  return false;
}

template <>
inline bool FrameGenerator::get<String>(const String &name, String &value) const
{
  SerializablePtr param;
  
  if(!_getParam(name, param))
    return false;
  
  SerializableString *p = dynamic_cast<SerializableString *>(param.get());
  
  if(p)
  {
    value = *p;
    return true;
  }
  return false;
}


class VOXEL_EXPORT DepthFrameGenerator: public FrameGenerator
{
public:
  DepthFrameGenerator(GeneratorIDType id, int frameType, uint8_t majorVersion, uint8_t minorVersion): FrameGenerator(id, frameType, majorVersion, minorVersion) {}
  virtual bool setProcessedFrameGenerator(FrameGeneratorPtr &p) = 0;
  
  virtual ~DepthFrameGenerator() {}
};

typedef Ptr<DepthFrameGenerator> DepthFrameGeneratorPtr;
  
}

#endif