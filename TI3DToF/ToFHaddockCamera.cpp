/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "ToFHaddockCamera.h"
#include <Configuration.h>
#include <ParameterDMLParser.h>

namespace Voxel
{
  
namespace TI
{

ToFHaddockCamera::ToFHaddockCamera(const String &name, DevicePtr device): ToFCamera(name, device)
{
}


  
bool ToFHaddockCamera::_init()
{
  Configuration c;
  
  String name = "OPT9220A.dml"; // TODO: This needs to come from a configuration
  
  if(!c.getConfFile(name)) // true => name is now a proper path
  {
    logger(ERROR) << "ToFHaddockCamera: Failed to locate/read DML file '" << name << "'" << std::endl;
    return false;
  }
  
  ParameterDMLParser p(*_programmer, name);
  
  Vector<ParameterPtr> params;
  
  if(!p.getParameters(params))
  {
    logger(ERROR) << "ToFHaddockCamera: Could not read parameters from DML file '" << name << "'" << std::endl;
    return false;
  }
  
  for(auto &p: params)
  {
    if((p->address() >> 8) == 0) // bankId == 0
      p->setAddress((0x58 << 8) + p->address());
    else if((p->address() >> 8) == 3) // bankId == 3
      p->setAddress((0x5C << 8) + (p->address() & 0xFF));
  }
  
  return _addParameters(params);
}

bool ToFHaddockCamera::getFrameRate(FrameRate &r)
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, pixCount, sysClkFrequency;
  
  if(!get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  if(!get(PIX_CNT_MAX, pixCount) || !get(QUAD_CNT_MAX, quadCount) || !get(SUBFRAME_CNT_MAX, subFrameCount) || !get(SYS_CLK_FREQ, sysClkFrequency))
    return false;
  
  uint numerator = sysClkFrequency*1000000,
  denominator = pixCount*quadCount*subFrameCount;
  
  uint g = gcd(numerator, denominator);
  
  r.numerator = numerator/g;
  r.denominator = denominator/g;
  return true;
}

bool ToFHaddockCamera::setFrameRate(const FrameRate &r)
{
  bool pixCountSetFailed;
  
  uint quadCount, subFrameCount, sysClkFrequency, pixCount;
  
  if(!get(QUAD_CNT_MAX, quadCount) || !get(SUBFRAME_CNT_MAX, subFrameCount) || !get(SYS_CLK_FREQ, sysClkFrequency))
    return false;
  
  pixCount = (uint)(((long)r.denominator*sysClkFrequency*1000000)/((long)quadCount*subFrameCount*r.numerator));
  
  logger(DEBUG) << "ToFHaddockCamera: Setting " << PIX_CNT_MAX << " = " << pixCount << std::endl;
  
  if(!set(PIX_CNT_MAX, pixCount) || !get(PIX_CNT_MAX_SET_FAILED, pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  return true;
}

bool ToFHaddockCamera::getFrameSize(FrameSize &s)
{
  s.width = 160;
  s.height = 240;
  return true; // dummy to be coded later
}


bool ToFHaddockCamera::setFrameSize(const FrameSize &s)
{
  return true; // dummy to be coded later
}


bool ToFHaddockCamera::_initStartParams()
{
  return set(TG_EN, true) and 
         set<uint>(BLK_SIZE, 1024) and
         set(BLK_HEADER_EN, true) and
         set(OP_CS_POL, true) and
         set(FB_READY_EN, true);
}

bool ToFHaddockCamera::_processRawFrame(RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput)
{
  FrameSize s;
  
  RawDataFramePtr rawDataFrame = std::dynamic_pointer_cast<RawDataFrame>(rawFrameInput);
  
  if(!rawDataFrame)
  {
    logger(ERROR) << "ToFHaddockCamera: Input data frame is not of raw data type." << std::endl;
    return false;
  }
  
  if(!getFrameSize(s))
  {
    logger(ERROR) << "ToFHaddockCamera: Could not get the current frame size. Cannot convert raw data to ToF data" << std::endl;
    return false;
  }
  
  int bytesPerPixel, dataArrangeMode;
  if(!get(PIXEL_DATA_SIZE, bytesPerPixel) or !get(OP_DATA_ARRANGE_MODE, dataArrangeMode))
  {
    logger(ERROR) << "ToFHaddockCamera: Failed to read " << PIXEL_DATA_SIZE << " or " << OP_DATA_ARRANGE_MODE << std::endl;
    return false;
  }
  
  ToFRawFrameTemplate<uint16_t, uint8_t> *t;
  
  if(!rawFrameOutput)
  {
    t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
    rawFrameOutput = RawFramePtr(t);
  }
  else
  {
    t = dynamic_cast<ToFRawFrameTemplate<uint16_t, uint8_t> *>(rawFrameOutput.get());
    
    if(!t)
    {
      t = new ToFRawFrameTemplate<uint16_t, uint8_t>();
      rawFrameOutput = RawFramePtr(t);
    }
  }
  
  t->size = s;
  t->id = rawDataFrame->id;
  t->timestamp = rawDataFrame->timestamp;
  
  if(bytesPerPixel == 4)
  {
    if(rawDataFrame->data.size() < s.height*s.width*4)
    {
      logger(ERROR) << "ToFHaddockCamera: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << s.height*s.width*4 << std::endl;
      return false;
    }
    
    if(dataArrangeMode != 2 && dataArrangeMode != 0)
    {
      logger(ERROR) << "ToFHaddockCamera: Invalid op_data_arrange_mode = " << dataArrangeMode << " for pixel_data_size = " << bytesPerPixel << std::endl;
      return false;
    }
    
    t->_ambient.resize(s.width*s.height);
    t->_amplitude.resize(s.width*s.height);
    t->_phase.resize(s.width*s.height);
    t->_flags.resize(s.width*s.height);
    
    if(dataArrangeMode == 2)
    {
      auto index1 = 0, index2 = 0;
      
      uint16_t *data = (uint16_t *)rawDataFrame->data.data();
      
      for (auto i = 0; i < s.height; i++) 
      {
        for (auto j = 0; j < s.width/8; j++) 
        {
          index1 = i*s.width*2 + j*16;
          index2 = i*s.width + j;
          
          //logger(INFO) << "i = " << i << ", j = " << j << ", index1 = " << index1 << ", index2 = " << index2 << std::endl;
          
          for (auto k = 0; k < 8; k++) 
          {
            t->_amplitude[index2 + k] = data[index1 + k] & 0x0FFF;
            t->_ambient[index2 + k] = (data[index1 + k] & 0xF000) >> 12;
            
            t->_phase[index2 + k] = data[index1 + k + 8] & 0x0FFF;
            t->_flags[index2 + k] = (data[index1 + k + 8] & 0xF000) >> 12;
          }
        }
      }
    }
    else // dataArrangeMode == 0
    {
      auto index1 = 0, index2 = 0;
      
      uint16_t *data = (uint16_t *)rawDataFrame->data.data();
      
      for (auto i = 0; i < s.height; i++) 
      {
        for (auto j = 0; j < s.width; j++) 
        {
          index1 = i*s.width*2 + j*2;
          index2 = i*s.width + j;
          t->_amplitude[index2] = data[index1] & 0x0FFF;
          t->_ambient[index2] = (data[index1] & 0xF000) >> 12;
          
          t->_phase[index2] = data[index1] & 0x0FFF;
          t->_flags[index2] = (data[index1] & 0xF000) >> 12;
        }
      }
    }
  }
  else if(bytesPerPixel == 2)
  {
    if(dataArrangeMode != 0)
    {
      logger(ERROR) << "ToFHaddockCamera: " << OP_DATA_ARRANGE_MODE << " is expected to be zero, but got = " << dataArrangeMode << " for " << PIXEL_DATA_SIZE << " = " << bytesPerPixel << std::endl;
      return false;
    }
    
    if(rawDataFrame->data.size() < s.height*s.width*2)
    {
      logger(ERROR) << "ToFHaddockCamera: Incomplete raw data size = " << rawDataFrame->data.size() << ". Required size = " << s.height*s.width*2 << std::endl;
      return false;
    }
    
    t->_ambient.resize(s.width*s.height);
    t->_amplitude.resize(s.width*s.height);
    t->_phase.resize(s.width*s.height);
    t->_flags.resize(s.width*s.height);
    
    auto index = 0;
    
    uint16_t *data = (uint16_t *)rawDataFrame->data.data();
    
    for (auto i = 0; i < s.height; i++) 
    {
      for (auto j = 0; j < s.width; j++) 
      {
        index = i*s.width + j;
        t->_phase[index] = data[index] & 0x0FFF;
        t->_amplitude[index] = (data[index] & 0xF000) >> 12;
        
        t->_ambient[index] = 0;
        t->_flags[index] = 0;
      }
    }
  }
  else
  {
    logger(ERROR) << "ToFHaddockCamera: Don't know to handle " << PIXEL_DATA_SIZE << " = " << bytesPerPixel << std::endl;
    return false;
  }
  
  bool histogramEnabled;
  if(!get(HISTOGRAM_EN, histogramEnabled))
  {
    logger(ERROR) << "ToFHaddockCamera: Failed to find whether histogram is enabled or not" << std::endl;
    return false;
  }
  
  if(!histogramEnabled)
    return true; // No histogram data
  
  if(rawDataFrame->data.size() < s.height*s.width*bytesPerPixel + 96)
  {
    logger(ERROR) << "ToFHaddockCamera: Histogram is enabled but raw data has less than 96 bytes at the end. Raw data size = " << rawDataFrame->data.size() 
      << ", bytes in frame = " << s.height*s.width*bytesPerPixel << std::endl;
    return false;
  }
  
  uint8_t *data = rawDataFrame->data.data() + s.height*s.width*bytesPerPixel;
  
  t->_histogram.resize(48); // 48 elements of 16-bits each
  
  memcpy((uint8_t *)t->_histogram.data(), data, 96);
  
  return true;
}
 
}
}