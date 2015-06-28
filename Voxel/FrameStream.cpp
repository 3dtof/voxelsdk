/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "FrameStream.h"

#include "Logger.h"

#include "CameraSystem.h"
#include "DepthCamera.h"

#include <FrameGenerator.h>

namespace Voxel
{
  
FrameStreamWriter::FrameStreamWriter(const String &filename, GeneratorIDType processedRawFrameGeneratorID, 
                                       GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID)
  :_stream(_internalStream)
{
  _stream.open(filename, std::ios::binary | std::ios::out);
  
  _init(processedRawFrameGeneratorID, depthFrameGeneratorID, pointCloudFrameGeneratorID);
}

bool FrameStreamWriter::_init(GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID)
{
  _frameCount = 0;
  
  if(!_stream.good())
  {
    logger(LOG_ERROR) << "FrameStreamWriter: Failed to open file." << std::endl;
    return false;
  }
  
  _header.generatorIDs[0] = processedRawFrameGeneratorID;
  _header.generatorIDs[1] = depthFrameGeneratorID;
  _header.generatorIDs[2] = pointCloudFrameGeneratorID;
  
  if(!_writeHeader())
  {
    logger(LOG_ERROR) << "FrameStreamWriter: Failed to write stream header." << std::endl;
    return false;
  }
  return true;
}


FrameStreamWriter::FrameStreamWriter(OutputFileStream &stream, GeneratorIDType processedRawFrameGeneratorID, 
                                     GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID)
:_stream(stream)
{
  _init(processedRawFrameGeneratorID, depthFrameGeneratorID, pointCloudFrameGeneratorID);
}

bool FrameStreamWriter::write(Voxel::FramePtr rawUnprocessed)
{
  Lock<Mutex> _(_mutex);
  
  if(!isStreamGood())
    return false;
  
  if((rawUnprocessed && !rawUnprocessed->serialize(_rawpacket.object)))
  {
    logger(LOG_ERROR) << "FrameStreamWriter: Failed to serialize frame." << std::endl;
    return false;
  }
  
  _rawpacket.type = FrameStreamPacket::PACKET_DATA;
  _rawpacket.size = _rawpacket.object.size();
  
  return _rawpacket.write(_stream);
}

bool FrameStreamWriter::_writeHeader()
{
  _header.version[0] = 0;
  _header.version[1] = 1;
  
  _stream.write(_header.version, 2);
  
  _stream.write((const char *)_header.generatorIDs, sizeof(GeneratorIDType)*3);
  
  return true;
}

// NOTE: This assumes that caller has populated _generatorConfigSubPacket.config
bool FrameStreamWriter::writeGeneratorConfiguration(uint frameType)
{
  Lock<Mutex> _(_mutex);
  
  if(!isStreamGood())
    return false;
  
  _generatorConfigSubPacket.frameType = frameType;
  _generatorConfigSubPacket.size = _generatorConfigSubPacket.config.size();
  
  _configPacket.type = FrameStreamPacket::PACKET_GENERATOR_CONFIG;
  _generatorConfigSubPacket.write(_configPacket.object);
  _configPacket.size = _configPacket.object.size();
  
  return _configPacket.write(_stream);
}

bool FrameStreamWriter::close()
{
  Lock<Mutex> _(_mutex);
  
  if(_stream.is_open())
    _stream.close();
  
  return !_stream.fail();
}

bool GeneratorConfigurationSubPacket::read(SerializedObject &object)
{
  if(object.size() < sizeof(size) + sizeof(frameType))
    return false;
  
  object.get((char *)&frameType, sizeof(frameType));
  object.get((char *)&size, sizeof(size));
  
  if(object.size() < sizeof(size) + sizeof(frameType) + size)
    return false;
  
  config.resize(size);
  
  object.get((char *)config.getBytes().data(), size);
  
  return true;
}

bool GeneratorConfigurationSubPacket::write(SerializedObject &object)
{
  object.resize(sizeof(size) + sizeof(frameType) + size);
  
  object.put((const char *)&frameType, sizeof(frameType));
  object.put((const char *)&size, sizeof(size));
  object.put((const char *)config.getBytes().data(), size);
  
  return true;
}


FrameStreamReader::FrameStreamReader(InputFileStream &stream, CameraSystem &sys):_stream(stream), _sys(sys), frames(4)
{
  _init();
}

FrameStreamReader::FrameStreamReader(const String &fileName, CameraSystem &sys): _stream(_internalStream), _sys(sys), frames(4)
{
  _stream.open(fileName, std::ios::binary | std::ios::in);
  
  if(!_stream.good())
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to open file '" << fileName << "'" << std::endl;
  }
  else
    _init();
}

bool FrameStreamReader::_init()
{
  if(!_stream.good())
    return false;
  
  _currentFrameIndex = _currentPacketIndex = 0;
  
  _stream.read(_header.version, 2);
  _stream.read((char *)&_header.generatorIDs, sizeof(GeneratorIDType)*3);
  
  if(_stream.fail() | _stream.bad())
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to read frame headers." << std::endl;
    return false;
  }
  
  if(!_sys.getFrameGenerator(DepthCamera::FRAME_RAW_FRAME_PROCESSED, _header.generatorIDs[0], _frameGenerator[0]) ||
  !_sys.getFrameGenerator(DepthCamera::FRAME_DEPTH_FRAME, _header.generatorIDs[1], _frameGenerator[1]) ||
  !_sys.getFrameGenerator(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, _header.generatorIDs[2], _frameGenerator[2]))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to get necessary frame generators to read stream." << std::endl;
    return false;
  }
  
  DepthFrameGeneratorPtr p = std::dynamic_pointer_cast<DepthFrameGenerator>(_frameGenerator[1]);
  
  if(!p || !p->setProcessedFrameGenerator(_frameGenerator[0]))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Could not initialize depth frame generator" << std::endl;
    return false;
  }
  
  _allPacketOffsets.reserve(500);
  _dataPacketLocation.reserve(500);
  _configPacketLocation.reserve(500);
  
  while(!_stream.eof())
  {
    uint offset = _stream.tellg();
    if(!_dataPacket.readHeader(_stream))
      break;
    
    if(!_dataPacket.verifyMagic())
    {
      logger(LOG_ERROR) << "FrameStreamReader: Got invalid packet at offset = " << _stream.tellg() << std::endl;
      continue;
    }
    
    if(_dataPacket.type == FrameStreamPacket::PACKET_DATA)
    {
      _allPacketOffsets.push_back(offset);
      _dataPacketLocation.push_back(_allPacketOffsets.size() - 1);
      _stream.seekg(_dataPacket.size, std::ios::cur);
    }
    else if(_dataPacket.type == FrameStreamPacket::PACKET_GENERATOR_CONFIG)
    {
      _allPacketOffsets.push_back(offset);
      _configPacketLocation.push_back(_allPacketOffsets.size() - 1);
      _stream.seekg(_dataPacket.size, std::ios::cur);
    }
    else
    {
      logger(LOG_ERROR) << "FrameStreamReader: Got invalid packet type = " << _dataPacket.type << std::endl;
    }
  }

  _stream.clear();
  if(_allPacketOffsets.size())
    _stream.seekg(_allPacketOffsets[0], std::ios::beg);
  
  logger(LOG_DEBUG) << "stream state: fail = " << _stream.fail() << ", bad = " << _stream.bad() << ", eof = " << _stream.eof() << std::endl;
  
  readNext(); // Read initial frame parameters
  seekTo(0);
  
  return true;
}

bool FrameStreamReader::_getPacket(size_t packetIndex, FrameStreamPacket &packet)
{
  if(packetIndex < 0 || packetIndex >= _allPacketOffsets.size())
    return false;
  
  _stream.seekg(_allPacketOffsets[packetIndex], std::ios::beg);
  
  if(!_stream.good())
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to move the stream get position" << std::endl;
    return false;
  }
  
  if(!packet.read(_stream))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to read packet with index = " << packetIndex << std::endl;
    return false;
  }
  
  if(!packet.verifyMagic())
  {
    logger(LOG_ERROR) << "FrameStreamReader: Found packet with invalid magic string" << std::endl;
    return false;
  }
  
  return true;
}

bool FrameStreamReader::_readConfigPacket(size_t packetIndex)
{
  if(!_getPacket(packetIndex, _configPacket))
    return false;
  
  if(_configPacket.type != FrameStreamPacket::PACKET_GENERATOR_CONFIG)
  {
    logger(LOG_ERROR) << "FrameStreamReader: Don't know how to handle config packet of type = '" << _configPacket.type << "'. Skipping it."<< "'" << std::endl;
    return false;
  }
  
  if(!_configSubPacket.read(_configPacket.object))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Could not get extract the configuration for config packet of type = '" << _configPacket.type << "'. Skipping it."<< "'" << std::endl;
    return false;
  }
  
  if(_configSubPacket.frameType > 3)
  {
    logger(LOG_ERROR) << "FrameStreamReader: Got configuration for unknown frame type = '" << _configSubPacket.frameType << "'" << std::endl;
    return false;
  }
  
  if(!_frameGenerator[_configSubPacket.frameType - 1]->readConfiguration(_configSubPacket.config))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to read configuration for frame type = '" << (uint)_configSubPacket.frameType << "'. Skipping it."<< "'" << std::endl;
    return false;
  }
  
  return true;
}



bool FrameStreamReader::readNext()
{
  if(_currentFrameIndex >= size())
    return false;
  
  size_t nextIndex = _dataPacketLocation[_currentFrameIndex];
  
  if(nextIndex > _currentPacketIndex) // some non-data packets in between?
  {
    for(; _currentPacketIndex < nextIndex; _currentPacketIndex++)
    {
      _readConfigPacket(_currentPacketIndex);
    }
  }
  
  RawDataFrame *r = dynamic_cast<RawDataFrame *>(frames[0].get());
  if(!r)
  {
    r = new RawDataFrame();
    frames[0] = FramePtr(r);
  }
  
  if(!_getPacket(_currentPacketIndex, _dataPacket))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to get data packet at index = " << _currentFrameIndex << std::endl;
    return false;
  }
  
  _currentFrameIndex++;
  _currentPacketIndex++;
  
  if(!r->deserialize(_dataPacket.object))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to deserialize data packet at index = " << _currentFrameIndex << std::endl;
    return false;
  }
  
  if(!_frameGenerator[0]->generate(frames[0], frames[1]) ||
    !_frameGenerator[1]->generate(frames[1], frames[2]) ||
    !_frameGenerator[2]->generate(frames[2], frames[3]))
  {
    logger(LOG_ERROR) << "FrameStreamReader: Failed to process and generate subsequent frame types at index = " << _currentFrameIndex << std::endl;
    return false;
  }
    
  return true;
}

bool FrameStreamReader::close()
{
  if(_stream.is_open())
    _stream.close();
  
  return !_stream.fail();
}

// TODO: Handle config changes in between seek positions
bool FrameStreamReader::seekTo(size_t position)
{
  if(position >= _dataPacketLocation.size())
    return false;
  
  if(position == 0)
    _currentFrameIndex = _currentPacketIndex = 0;
  else
  {
    _currentFrameIndex = position;
    _currentPacketIndex = _dataPacketLocation[position];
  }
  
  return true;
}
  
}