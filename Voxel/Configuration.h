/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_CONFIGURATION_H
#define VOXEL_CONFIGURATION_H

#include "Common.h"

namespace Voxel
{

class Configuration
{
protected:
  struct _Path
  {
    String standardPath;
    String environmentVariable;
  };
  
  Map<String, _Path> _pathTypes = {
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
  
  bool _getPaths(const String &type, Vector<String> &paths);
  
  bool _get(const String &type, String &name);
  
public:
  inline bool getFirmwarePaths(Vector<String> &paths) { return _getPaths("firmware", paths); }
  inline bool getConfPaths(Vector<String> &paths) { return _getPaths("conf", paths); }
  inline bool getLibPaths(Vector<String> &paths) { return _getPaths("lib", paths); }
  
  /// Updates "name" to full path
  inline bool getConfFile(String &name) { return _get("conf", name); }
  inline bool getFirmwareFile(String &name) { return _get("firmware", name); }
  inline bool geLibFile(String &name) { return _get("lib", name); }
  
};

}

#endif // CONFIGURATION_H
