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
  
  if(!get("pix_cnt_max_set_failed", pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  if(!get("pix_cnt_max", pixCount) || !get("quad_cnt_max", quadCount) || !get("uframe_cnt_max", subFrameCount) || !get("sys_clk_freq", sysClkFrequency))
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
  
  if(!get("quad_cnt_max", quadCount) || !get("uframe_cnt_max", subFrameCount) || !get("sys_clk_freq", sysClkFrequency))
    return false;
  
  pixCount = (uint)(((long)r.denominator*sysClkFrequency*1000000)/((long)quadCount*subFrameCount*r.numerator));
  
  logger(DEBUG) << "ToFHaddockCamera: Setting pix_cnt_max = " << pixCount << std::endl;
  
  if(!set("pix_cnt_max", pixCount) || !get("pix_cnt_max_set_failed", pixCountSetFailed) || pixCountSetFailed)
    return false;
  
  return true;
}

bool ToFHaddockCamera::getFrameSize(FrameSize &s)
{
  return true; // dummy to be coded later
}


bool ToFHaddockCamera::setFrameSize(const FrameSize &s)
{
  return true; // dummy to be coded later
}


bool ToFHaddockCamera::_initStartParams()
{
  return set("tg_en", true) and 
         set<uint>("blk_size", 1024) and
         set("blk_header_en", true) and
         set("op_cs_pol", true) and
         set("fb_ready_en", true);
}

  
}
}