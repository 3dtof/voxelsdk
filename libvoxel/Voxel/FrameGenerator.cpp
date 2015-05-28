/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "FrameGenerator.h"
#include "FrameStream.h"

namespace Voxel
{

bool FrameGenerator::writeConfiguration()
{
  if(!_frameStreamWriter)
    return true;
  
  auto &p = _frameStreamWriter->getConfigObject();
  
  p.resize(_frameGeneratorParameters.serializedSize());
  
  if(!_frameGeneratorParameters.write(p))
    return false;
  
  if(!_onWriteConfiguration())
    return false;
  
  return _frameStreamWriter->writeGeneratorConfiguration(_frameType);
}

bool FrameGenerator::readConfiguration(SerializedObject &object)
{
  if(!_frameGeneratorParameters.read(object))
    return false;
  
  uint32_t version;
  
  if(!get("version", version))
    return false;
  
  _majorVersion = (version & 0xFF00) >> 8;
  _minorVersion = (version & 0xFF);
  
  if(!_onReadConfiguration())
    return false;
  
  return true;
}

}