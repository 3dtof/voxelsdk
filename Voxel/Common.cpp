/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Common.h"
#include <sstream>
#include <iomanip>

#include <sys/time.h>

namespace Voxel
{

String getHex(uint16_t value)
{
  std::ostringstream s;
  s << std::hex << std::setw(4) << std::setfill('0') << value;
  return s.str();
}

void split(const String &str, const char delimiter, Vector<String> &split)
{
  int pos = str.find(delimiter, 0);
  int previous = 0;
  
  while (pos != String::npos)
  {
    split.push_back(str.substr(previous, pos));
    
    previous = pos + 1;
    pos = str.find(delimiter, previous);
  }
  
  split.push_back(str.substr(previous, pos));
}
  
}