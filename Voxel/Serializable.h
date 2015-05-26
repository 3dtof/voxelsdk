/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_SERIALIZABLE_H
#define VOXEL_SERIALIZABLE_H

#include "Common.h"
#include "SerializedObject.h"

#include "Logger.h"

namespace Voxel
{
  
class Serializable
{
public:
  virtual size_t serializedSize() const = 0;
  virtual bool read(SerializedObject &object) = 0;
  virtual bool write(SerializedObject &object) const = 0;
  
  virtual ~Serializable() {}
};

typedef Ptr<Serializable> SerializablePtr;

template <typename T>
class BasicSerializable: public Serializable
{
public:
  T value;
  
  BasicSerializable() {}
  BasicSerializable(const T &v) { value = v; }
  
  size_t serializedSize() const { return sizeof(T); }
  
  bool read(SerializedObject &object)
  {
    return object.get((char *)&value, sizeof(T)) == sizeof(T);
  }
  
  bool write(SerializedObject &object) const
  {
    return object.put((const char *)&value, sizeof(T)) == sizeof(T);
  }
  
  virtual ~BasicSerializable() {}
};

typedef BasicSerializable<uint32_t> SerializableUnsignedInt;
typedef BasicSerializable<int32_t> SerializableSignedInt;
typedef BasicSerializable<float> SerializableFloat;
typedef Ptr<SerializableUnsignedInt> SerializableUnsignedIntPtr;
typedef Ptr<SerializableSignedInt> SerializableSignedIntPtr;
typedef Ptr<SerializableFloat> SerializableFloatPtr;

class SerializableString: public Serializable, public String
{
public:
  SerializableString() {}
  SerializableString(const String &other): String(other) {}
  
  size_t serializedSize() const { return sizeof(uint32_t) + size(); }
  
  bool read(SerializedObject &object)
  {
    uint32_t size;
    if(object.get((char *)&size, sizeof(uint32_t)) != sizeof(uint32_t))
      return false;
    
    resize(size);
    
    if(object.get((char *)data(), sizeof(char)*size) != sizeof(char)*size)
      return false;
    
    return true;
  }
  
  bool write(SerializedObject &object) const
  {
    uint32_t size = this->size();
    if(object.put((const char *)&size, sizeof(uint32_t)) != sizeof(uint32_t))
      return false;
    
    if(object.put((const char *)data(), sizeof(char)*size) != sizeof(char)*size)
      return false;
    
    return true;
  }
  
  virtual ~SerializableString() {}
};

typedef Ptr<SerializableString> SerializableStringPtr;

}

namespace std {
  template <>
  struct hash<Voxel::SerializableString> {
    std::size_t operator()(const Voxel::SerializableString &k) const
    {
      return hash<string>()(k);
    }
  };
}

namespace Voxel {

// NOTE: Key is expected to be derived from Serializable
template <typename Key>
class SerializableMap: public Map<Key, SerializablePtr>, Serializable
{
public:
  SerializableMap(Map<Key, SerializablePtr> &other): Map<Key, SerializablePtr>(other) {}
  SerializableMap(SerializableMap &other): Map<Key, SerializablePtr>(other) {}
  SerializableMap() {}
  
  virtual size_t serializedSize() const
  {
    uint32_t size = sizeof(uint32_t);
    
    for(auto &x: *this)
    {
      size += x.first.serializedSize();
      size += x.second->serializedSize();
    }
    
    return size;
  }
  
  virtual bool write(SerializedObject &object) const
  {
    uint32_t size = serializedSize() - sizeof(uint32_t);
    
    if(object.size() - object.currentPutOffset() < size)
      return false;
    
    if(object.put((const char *)&size, sizeof(uint32_t)) != sizeof(uint32_t))
      return false;
    
    
    for(auto &x: *this)
    {
      if(!x.first.write(object) || !x.second->write(object))
        return false;
    }
    return true;
  }
  
  virtual bool read(SerializedObject &object)
  {
    uint32_t size;
    if(object.get((char *)&size, sizeof(uint32_t)) != sizeof(uint32_t))
      return false;
    
    uint32_t g = object.currentGetOffset();
    
    Key k;
    
    while(object.currentGetOffset() - g < size)
    {
      if(!k.read(object))
        return false;
      
      auto x = this->find(k);
      
      if(x == this->end())
      {
        logger(LOG_WARNING) << "SerializableMap: Found unknown key '" << k << "'" << std::endl;
        continue;
      }
      
      if(!x->second->read(object))
        return false;
    }
    
    return object.currentGetOffset() == (g + size);
  }
  
  virtual ~SerializableMap() {}
};

class StringKeySerializableMap: public SerializableMap<SerializableString>
{
public:
  SerializablePtr &operator [](const String &key)
  {
    return SerializableMap<SerializableString>::operator[](SerializableString(key));
  }
  
  virtual ~StringKeySerializableMap() {}
};

typedef Ptr<StringKeySerializableMap> StringKeySerializableMapPtr;

}
#endif
