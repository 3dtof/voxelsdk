/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "DataPacket.h"

namespace Voxel
{

bool DataPacket::write(OutputStream &out)
{
  if(!out.good())
    return false;
  
  out.write(magic, 5);
  out.write((const char *)&type, sizeof(type));
  out.write((const char *)&size, sizeof(size));
  out.write((const char *)object.getBytes().data(), size);
  
  if(out.fail() | out.bad())
    return false;
  
  return true;
}

bool DataPacket::write(SerializedObject &out)
{
  if(out.size() - out.currentPutOffset() < 5 + sizeof(type) + sizeof(size) + size)
    return false;
  
  out.put(magic, 5);
  out.put((const char *)&type, sizeof(type));
  out.put((const char *)&size, sizeof(size));
  out.put((const char *)object.getBytes().data(), size);
  
  return true;
}


bool DataPacket::read(InputStream &in)
{
  if(!readHeader(in))
    return false;
  
  object.resize(size);
  in.read((char *)object.getBytes().data(), size);
  
  if(in.fail() | in.bad())
    return false;
  
  return true;
}

bool DataPacket::read(SerializedObject &in)
{
  if(!readHeader(in))
    return false;
  
  if((in.size() - in.currentGetOffset()) < size)
    return false;
  
  object.resize(size);
  in.get((char *)object.getBytes().data(), size);
  
  return true;  
}


bool DataPacket::readHeader(InputStream &in)
{
  if(!in.good())
    return false;
  
  in.read(magic, 5);
  in.read((char *)&type, sizeof(type));
  in.read((char *)&size, sizeof(size));
  
  if(in.fail() | in.bad())
    return false;
  
  return true;
}

bool DataPacket::readHeader(SerializedObject &in)
{
  if((in.size() - in.currentGetOffset()) < 5 + sizeof(type) + sizeof(size))
    return false;
  
  in.get(magic, 5);
  in.get((char *)&type, sizeof(type));
  in.get((char *)&size, sizeof(size));
  
  return true;
}


}