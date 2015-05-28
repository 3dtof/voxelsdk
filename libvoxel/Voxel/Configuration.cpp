/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Configuration.h"
#include "Logger.h"

#include <stdlib.h>
#include <fstream>

#include <string.h>
#include <stdio.h>

#ifdef LINUX
#define DIR_SEP "/"
#define PATH_SEP ':'
#elif defined(WINDOWS)
#define DIR_SEP "\\"
#define PATH_SEP ';'
#endif


#define CAMERA_PROFILES_SECTION "camera_profiles"

namespace Voxel
{

const Map<String, Configuration::_Path> Configuration::_pathTypes = {
  { "firmware",
    {
#ifdef LINUX
      "/lib/firmware/voxel",
#elif defined(WINDOWS)
      "C:\\Program Files\\VoxelCommon\\fw",
#endif
      "VOXEL_FW_PATH"
    }
  },
  { "lib",
    {
#ifdef LINUX
      "/usr/lib/voxel", 
#elif defined(WINDOWS)
      "C:\\Program Files\\VoxelCommon\\lib",
#endif
      "VOXEL_LIB_PATH"
    }
  },
  { "conf",
    {
#ifdef LINUX
      "/etc/voxel",
#elif defined(WINDOWS)
      "C:\\Program Files\\VoxelCommon\\conf",
#endif
      "VOXEL_CONF_PATH"
    }
  }
};

bool Configuration::_addPath(const String &type, const String &path)
{
  auto t = _pathTypes.find(type);

  if(t == _pathTypes.end())
    return false;

  const _Path &pt = t->second;

  char *p = getenv(pt.environmentVariable.c_str());

#ifdef WINDOWS
  if(p != 0)
    return _putenv_s(pt.environmentVariable.c_str(), (path + PATH_SEP + p).c_str()) == 0;
  else
    return _putenv_s(pt.environmentVariable.c_str(), path.c_str()) == 0;
#elif defined(LINUX)
  if(p != 0)
    return setenv(pt.environmentVariable.c_str(), (path + PATH_SEP + p).c_str(), 1) == 0;
  else
    return setenv(pt.environmentVariable.c_str(), path.c_str(), 1) == 0;
#endif
}

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
    
    split(p1, PATH_SEP, splits);
    
    paths.reserve(paths.size() + splits.size());
    paths.insert(paths.begin(), splits.begin(), splits.end()); // Insert at the beginning to override standard path
  }
  
  if(logger.getDefaultLogLevel() >= LOG_DEBUG) 
  {
    for(auto i = 0; i < paths.size(); i++)
    {
      logger(LOG_DEBUG) << paths[i];
      if(i < paths.size() - 1)
        logger(LOG_DEBUG) << PATH_SEP;
    }
    logger(LOG_DEBUG) << std::endl;
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

bool ConfigSet::isPresent(const String &name) const
{
  if(params.find(name) == params.end())
    return false;
  return true;
}


bool ConfigurationFile::isPresent(const String &section, const String &name) const
{
  if(configs.find(section) == configs.end())
    return false;
  
  const ConfigSet &set = configs.at(section);
  
  return set.isPresent(name);
}

bool ConfigurationFile::getConfigSet(const String &section, const ConfigSet *&configSet) const
{
  if(configs.find(section) != configs.end())
  {
    configSet = &configs.at(section);
    return true;
  }
  
  return false;
}

bool ConfigSet::_get(const String &name, String &value) const
{
  if(params.find(name) != params.end())
  {
    value = params.at(name);
    return true;
  }
  return false;
}


bool ConfigurationFile::_get(const String &section, const String &name, String &value) const
{
  if(configs.find(section) != configs.end())
  {
    const ConfigSet &set = configs.at(section);
    
    return set._get(name, value);
  }
  
  return false;
}

String ConfigSet::get(const String &name) const
{
  String value;
  if(!_get(name, value))
    return "";
  else
    return value;
}


String ConfigurationFile::get(const String &section, const String &name) const
{
  String value;
  
  if(!_get(section, name, value))
    return "";
  else
    return value;
}

int ConfigSet::getInteger(const String &name) const
{
  String s = get(name);
  
  if(!s.size())
    return 0;
  
  return atoi(s.c_str());
}


int ConfigurationFile::getInteger(const String &section, const String &name) const
{
  String s = get(section, name);
  
  if(!s.size())
    return 0;
  
  return atoi(s.c_str());
}

float ConfigSet::getFloat(const String &name) const
{
  String s = get(name);
  
  if(!s.size())
    return 0.0f;
  
  return atof(s.c_str());
}


float ConfigurationFile::getFloat(const String &section, const String &name) const
{
  String s = get(section, name);
  
  if(!s.size())
    return 0.0f;
  
  return atof(s.c_str());
}

bool ConfigSet::getBoolean(const String &name) const
{
  String s = get(name);
  
  if(!s.size())
    return false;
  
  if(s == "true" || s == "True" || s == "1")
    return true;
  else
    return false;
}


bool ConfigurationFile::getBoolean(const String &section, const String &name) const
{
  String s = get(section, name);
  
  if(!s.size())
    return false;
  
  if(s == "true" || s == "True" || s == "1")
    return true;
  else
    return false;
}


bool ConfigurationFile::read(const String &filename)
{
  char buffer[2048];
  char name[1000];
  char value[1000];
  
  ConfigSet *currentSet = 0;
  
  Configuration c;
  String f = filename;
  
  configs.clear();
  
  if(!c.getConfFile(f))
  {
    logger(LOG_ERROR) << "ConfigurationFile: Failed to get configuration file '" << f << "'" << std::endl;
    return false;
  }
  
  _fileName = f;
  
  InputFileStream fin(f, std::ios::binary | std::ios::in);
  
  if (!fin.good())
  {
    logger(LOG_ERROR) << "ConfigurationFile: Could not read file '" << filename << "'" << std::endl;
    return false;
  }
  
  while(fin.good())
  {
    fin.getline(buffer, 2048);
    
    if(strlen(buffer) < 2)
    {
      // ignore this line - nothing to do
    }
    else if(buffer[0] == '[') // new section
    {
      int nret = sscanf(buffer, "[%[^]]]", name);
      
      if(nret == 1)
        currentSet = &configs[name];
      else
        return false;
    }
    else if(buffer[0] != '#') // allow comments
    {
      int nret = sscanf(buffer, "%[^=] = %[^\t\r\n]", name, value);
      
      if(nret >= 1 && currentSet != 0)
      { 
        String n = name;
        trim(n);
        String v;
        if(nret == 2)
        {
          v = value;
          trim(v);
        }
        (*currentSet).paramNames.push_back(n);
        (*currentSet).params[n] = v;
      }
      else
        return false;
    }
  }
  
  return true;
}

bool MainConfigurationFile::read(const String &configFile)
{
  if(!ConfigurationFile::read(configFile))
    return false;
  
  _cameraProfiles.clear();
  _cameraProfileNames.clear();
  
  const ConfigSet *cameraProfiles;
  
  if(!getConfigSet(CAMERA_PROFILES_SECTION, cameraProfiles))
  {
    logger(LOG_ERROR) << "Could not find 'camera_profiles' section in " << configFile << std::endl;
    return false;
  }
  
  for(auto i = 0; i < cameraProfiles->paramNames.size(); i++)
  {
    if(cameraProfiles->paramNames[i] == "default")
    {
      _defaultCameraProfileName = cameraProfiles->params.at(cameraProfiles->paramNames[i]);
      continue;
    }
    
    const String &profileConfigFile = cameraProfiles->params.at(cameraProfiles->paramNames[i]);
    
    if(!_cameraProfiles[cameraProfiles->paramNames[i]].read(profileConfigFile))
    {
      logger(LOG_ERROR) << "Could not read " << profileConfigFile << std::endl;
      return false;
    }
    
    _cameraProfileNames.push_back(cameraProfiles->paramNames[i]);
  }
  
  if(_defaultCameraProfileName.size() == 0)
  {
    logger(LOG_ERROR) << "Default profile not defined in " << configFile << std::endl;
    return false;
  }
  
  if(!setCurrentCameraProfile(_defaultCameraProfileName))  
  {
    logger(LOG_ERROR) << "Default profile is defined but not found in " << configFile << std::endl;
    return false;
  }
  
  return true;
}

ConfigurationFile *MainConfigurationFile::getCameraProfile(const String &profileName)
{
  if(_cameraProfiles.find(profileName) != _cameraProfiles.end())
  {
    return &_cameraProfiles.at(profileName);
  }
  return 0;
}

ConfigurationFile *MainConfigurationFile::getDefaultCameraProfile()
{
  return getCameraProfile(_defaultCameraProfileName);
}

bool MainConfigurationFile::setCurrentCameraProfile(const String &profileName)
{
  if(_cameraProfiles.find(profileName) != _cameraProfiles.end())
  {
    _currentCameraProfileName = profileName;
    _currentCameraProfile = &_cameraProfiles.at(profileName);
    return true;
  }
  
  return false;
}

String MainConfigurationFile::get(const String &section, const String &name) const
{
  String value;
  
  if(_currentCameraProfile && _currentCameraProfile->_get(section, name, value))
  {
    return value;
  }
  
  if(ConfigurationFile::_get(section, name, value))
    return value;
  
  return "";
}

bool MainConfigurationFile::isPresent(const String &section, const String &name) const
{
  if(_currentCameraProfile && _currentCameraProfile->isPresent(section, name))
    return true;
  
  if(ConfigurationFile::isPresent(section, name))
    return true;
  
  return false;
}




}