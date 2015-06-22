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


#define CAMERA_PROFILES "camera_profiles"

namespace Voxel
{

const Map<String, Configuration::_Path> Configuration::_pathTypes = {
  { "fw", // Firmwares
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

bool Configuration::_getLocalPath(const String &type, String &path)
{
  // Adding local path ~/.Voxel/<type>
  // Works typically for Windows
  char *homeDrive = getenv("HOMEDRIVE");
  char *homePath = getenv("HOMEPATH");
  if(homeDrive != 0 && homePath != 0)
  {
    String s = homeDrive;
    s += homePath;
    
    s += DIR_SEP;
    s += ".Voxel";
    s += DIR_SEP;
    s += type;
    
    path = s;
    return true;
  }
  
  // Adding local path ~/.Voxel/<type>
  // Works typically for Linux
  char *home = getenv("HOME");
  if(home != 0)
  {
    String s = home;
    
    s += DIR_SEP;
    s += ".Voxel";
    s += DIR_SEP;
    s += type;
    
    path = s;
    return true;
  }
  
  return false;
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
  
  String localPath;
  
  if(_getLocalPath(type, localPath))
    paths.insert(paths.begin(), localPath);
  
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
  bool checkParent = false;
  
  if(configs.find(section) == configs.end())
    checkParent = true;
  else
  {
    const ConfigSet &set = configs.at(section);
    
    if(!set.isPresent(name))
      checkParent = true;
    else
      return true;
  }
  
  if(checkParent)
  {
    if(section != "global" && _mainConfigurationFile && isPresent("global", "parent"))
    {
      int parentID = getInteger("global", "parent");
      
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(parentID);
      
      // TODO This does not handle circular references between profiles
      if(parentConfig)
        return parentConfig->isPresent(section, name);
    }
  }
  return false;
}

bool ConfigurationFile::getConfigSet(const String &section, const ConfigSet *&configSet) const
{
  if(configs.find(section) != configs.end())
  {
    configSet = &configs.at(section);
    return true;
  }
  else
  {
    if(section != "global" && _mainConfigurationFile && isPresent("global", "parent"))
    {
      int parentID = getInteger("global", "parent");
      
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(parentID);
      
      // TODO This does not handle circular references between profiles
      if(parentConfig)
        return parentConfig->getConfigSet(section, configSet);
    }
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
  int parentID;
  bool checkParent = false;
  
  if(configs.find(section) != configs.end())
  {
    const ConfigSet &set = configs.at(section);
    
    if(!set._get(name, value))
      checkParent = true;
    else
      return true;
  }
  else 
    checkParent = true;
  
  if(checkParent)
  {
    if(section != "global" && _mainConfigurationFile && isPresent("global", "parent"))
    {
      parentID = getInteger("global", "parent");
      
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(parentID);
      
      // TODO This does not handle circular references between profiles
      if(parentConfig)
        return parentConfig->_get(section, name, value);
    }
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
  
  String cameraProfiles;
  
  if(!isPresent("core", CAMERA_PROFILES))
  {
    logger(LOG_ERROR) << "Could not find 'camera_profiles' in " << configFile << std::endl;
    return false;
  }
  
  cameraProfiles = get("core", CAMERA_PROFILES);
  
  Vector<String> cp;
  split(cameraProfiles, ',', cp);
  
  for(auto i = 0; i < cp.size(); i++)
  {
    const String &profileConfigFile = trim(cp[i]);
    
    ConfigurationFile cf(this);
    int id;
    
    if(!cf.read(profileConfigFile))
    {
      logger(LOG_ERROR) << "MainConfigurationFile: Could not read " << profileConfigFile << std::endl;
      return false;
    }
    
    if(!cf.isPresent("global", "id"))
    {
      logger(LOG_ERROR) << "MainConfigurationFile: Could not get ID of profile " << profileConfigFile << std::endl;
      return false;
    }
    
    id = cf.getInteger("global", "id");
    
    if(_cameraProfiles.find(id) != _cameraProfiles.end())
    {
      logger(LOG_ERROR) << "MainConfigurationFile: A profile with ID = " << id << " already exists." << std::endl;
      return false;
    }
    
    String cameraProfileName;
    
    if(!cf.isPresent("global", "name"))
    {
      logger(LOG_ERROR) << "MainConfigurationFile: Could not get name of profile in " << profileConfigFile << std::endl;
      return false;
    }
    
    cameraProfileName = cf.get("global", "name");
    
    _cameraProfiles[id] = cf;
    _cameraProfileNames[id] = cameraProfileName;
  }
  
  if(!isPresent("core", "default_profile"))
  {
    logger(LOG_ERROR) << "MainConfigurationFile: Default profile not defined in " << configFile << std::endl;
    return false;
  }
  
  _defaultCameraProfileID = getInteger("core", "default_profile");
  
  if(_cameraProfiles.find(_defaultCameraProfileID) == _cameraProfiles.end())
  {
    logger(LOG_ERROR) << "MainConfigurationFile: Default profile not valid in " << configFile << std::endl;
    return false;
  }
  
  if(!setCurrentCameraProfile(_defaultCameraProfileID))  
  {
    logger(LOG_ERROR) << "MainConfigurationFile: Default profile is defined but not found in " << configFile << std::endl;
    return false;
  }
  
  return true;
}

ConfigurationFile *MainConfigurationFile::getCameraProfile(const int id)
{
  if(_cameraProfiles.find(id) != _cameraProfiles.end())
  {
    return &_cameraProfiles.at(id);
  }
  return 0;
}

ConfigurationFile *MainConfigurationFile::getDefaultCameraProfile()
{
  return getCameraProfile(_defaultCameraProfileID);
}

bool MainConfigurationFile::setCurrentCameraProfile(const int id)
{
  if(_cameraProfiles.find(id) != _cameraProfiles.end())
  {
    _currentCameraProfileID = id;
    _currentCameraProfile = &_cameraProfiles.at(id);
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

bool MainConfigurationFile::getCameraProfileName(const int id, String &cameraProfileName)
{
  if(_cameraProfileNames.find(id) != _cameraProfileNames.end())
  {
    cameraProfileName = _cameraProfileNames.at(id);
    return true;
  }
  
  return false; 
}




}