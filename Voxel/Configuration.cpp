/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Configuration.h"
#include "Logger.h"

#include <stdlib.h>
#include <fstream>

namespace Voxel
{
  
bool Configuration::_getPaths(const String &type, Vector<String> &paths)
{
  auto t = _pathTypes.find(type);
  
  if(t == _pathTypes.end())
    return false;
  
  _Path &pt = t->second;
  
  paths.clear();
  paths.push_back(pt.standardPath);
  
  char *p = getenv(pt.environmentVariable.c_str());
  
  if(p != 0)
  {
    String p1(p);
    
    Vector<String> splits;
    
    split(p1, ':', splits);
    
    paths.reserve(paths.size() + splits.size());
    paths.insert(paths.end(), splits.begin(), splits.end());
  }
  
  if(log.getDefaultLogLevel() >= DEBUG) 
  {
    for(auto i = 0; i < paths.size(); i++)
    {
      log(DEBUG) << paths[i];
      if(i < paths.size() - 1)
        log(DEBUG) << ":";
    }
    log(DEBUG) << endl;
  }
  
  return true;
}

bool Configuration::_get(const String &type, String &name)
{
  Vector<String> paths;
  if(!_getPaths(type, paths))
    return false;
  
  for(auto &p: paths)
  {
    std::ifstream f(p + DIR_SEP + name, std::ios::binary);
    
    if(f.good())
    {
      name = p + DIR_SEP + name;
      return true;
    }
  }
  return false;
}


}