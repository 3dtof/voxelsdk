/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Configuration.h"
#include "Logger.h"

#include <stdlib.h>
#include <fstream>

#ifdef LINUX
#define DIR_SEP "/"
#elif defined(WINDOWS)
#define DIR_SEP "\\"
#endif


namespace Voxel
{

const Map<String, Configuration::_Path> Configuration::_pathTypes = {
  { "firmware",
    {
      "/lib/firmware/voxel",
      "VOXEL_FW_PATH"
    }
  },
  { "lib",
    {
      "/usr/lib/voxel",
      "VOXEL_LIB_PATH"
    }
  },
  { "conf",
    {
      "/etc/voxel",
      "VOXEL_CONF_PATH"
    }
  }
};

bool Configuration::_getPaths(const String &type, Vector<String> &paths)
{
  auto t = _pathTypes.find(type);
  
  if(t == _pathTypes.end())
    return false;
  
  const _Path &pt = t->second;
  
  paths.clear();
  paths.push_back(pt.standardPath);
  
  char *p = getenv(pt.environmentVariable.c_str());
  
  if(p != 0)
  {
    String p1(p);
    
    Vector<String> splits;
    
    split(p1, ':', splits);
    
    paths.reserve(paths.size() + splits.size());
    paths.insert(paths.begin(), splits.begin(), splits.end()); // Insert at the beginning to override standard path
  }
  
  if(logger.getDefaultLogLevel() >= LOG_DEBUG) 
  {
    for(auto i = 0; i < paths.size(); i++)
    {
      logger(LOG_DEBUG) << paths[i];
      if(i < paths.size() - 1)
        logger(LOG_DEBUG) << ":";
    }
    logger(LOG_DEBUG) << endl;
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