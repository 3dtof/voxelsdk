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
  
  paths.insert(paths.begin(), ""); // Empty path for absolute file paths and also relative file paths
  
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
    if(section != "global" && _mainConfigurationFile && _parentID >= 0)
    {
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(_parentID);
      
      // TODO This does not handle circular references between profiles
      if(parentConfig)
      {
        return parentConfig->isPresent(section, name);
      }
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
    if(section != "global" && _mainConfigurationFile && _parentID >= 0)
    {
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(_parentID);
      
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
  Lock<Mutex> _(_mutex);
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
    if(section != "global" && _mainConfigurationFile && _parentID >= 0)
    {
      ConfigurationFile *parentConfig = _mainConfigurationFile->getCameraProfile(_parentID);
      
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
  
  return read(fin);
}

bool ConfigurationFile::read(InputStream &in)
{
  {
    Lock<Mutex> _(_mutex);
    
    char buffer[2048];
    char name[1000];
    char value[1000];
    
    ConfigSet *currentSet = 0;
    
    while(in.good())
    {
      in.getline(buffer, 2048);
      
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
  }
  
  if(isPresent("global", "id"))
    _id = getInteger("global", "id");
  
  if(isPresent("global", "parent"))
    _parentID = getInteger("global", "parent");
  
  if(isPresent("global", "name"))
    _profileName = get("global", "name");
  
  return true;
}

bool ConfigurationFile::write(OutputStream &out)
{
  Lock<Mutex> _(_mutex);
  
  for(auto c = configs.begin(); c != configs.end(); c++)
  {
    out << "[" << c->first << "]\n";
    
    for(auto i = 0; i < c->second.paramNames.size(); i++)
    {
      out << c->second.paramNames[i] << " = " << c->second.params[c->second.paramNames[i]] << std::endl;
    }
    out << std::endl;
  }
  return true;
}

bool ConfigurationFile::write(const String &configFile)
{
  Configuration c;
  String path;
  
  if(!c.getLocalConfPath(path))
  {
    logger(LOG_ERROR) << "ConfigurationFile: Failed to get configuration local path." << std::endl;
    return false;
  }
  
  if(configFile != "") // Use new file name or old one itself?
  {
    String f = configFile;
    _fileName = path + DIR_SEP + basename(f);
  }
  else
  {
    if(_fileName.compare(0, path.size(), path) != 0)
      _fileName = path + DIR_SEP + basename(_fileName);
  }
  
  OutputFileStream fout(_fileName, std::ios::binary | std::ios::out);
  
  if (!fout.good())
  {
    logger(LOG_ERROR) << "ConfigurationFile: Could not write file '" << configFile << "'" << std::endl;
    return false;
  }
  
  return write(fout);
}

bool ConfigSet::_set(const String &name, const String &value)
{
  String n = name, v = value;
  trim(n);
  trim(v);
  params[n] = v;
  
  bool present = false;
  
  for(auto i = 0; i < paramNames.size(); i++)
    if(paramNames[i] == n)
    {
      present = true;
      break;
    }
    
  if(!present)
    paramNames.push_back(name);
  
  return true;
}

bool ConfigSet::set(const String &name, const String &value)
{
  return _set(name, value);
}

bool ConfigSet::setBoolean(const String &name, bool value)
{
  if(value)
    return _set(name, "true");
  else
    return _set(name, "false");
}

bool ConfigSet::setFloat(const String &name, float value)
{
  OutputStringStream o;
  o << value;
  
  return _set(name, o.str());
}

bool ConfigSet::setInteger(const String &name, int value)
{
  OutputStringStream o;
  o << value;
  
  return _set(name, o.str());
}

bool ConfigurationFile::_set(const String &section, const String &name, const String &value)
{
  Lock<Mutex> _(_mutex);
  if(configs.find(section) == configs.end())
    configs[section]; // Create section
    
  ConfigSet &set = configs.at(section);
    
  return set._set(name, value);
}

bool ConfigurationFile::set(const String &section, const String &name, const String &value)
{
  if(section == "global" && name == "name")
    _profileName = value;
  
  return _set(section, name, value);
}

bool ConfigurationFile::setBoolean(const String &section, const String &name, bool value)
{
  if(value)
    return _set(section, name, "true");
  else
    return _set(section, name, "false");
}

bool ConfigurationFile::setFloat(const String &section, const String &name, float value)
{
  OutputStringStream s;
  s << value;
  
  return _set(section, name, s.str());
}

bool ConfigurationFile::setInteger(const String &section, const String &name, int value)
{
  OutputStringStream s;
  s << value;
  
  if(section == "global")
  {
    if(name == "id")
      _id = value;
    else if(name == "parent")
      _parentID = value;
  }
  
  return _set(section, name, s.str());
}

bool ConfigSet::remove(const String &name)
{
  if(params.find(name) != params.end())
  {
    params.erase(params.find(name));
    
    for(auto i = paramNames.begin(); i != paramNames.end(); i++)
      if(name == *i)
      {
        paramNames.erase(i);
        break;
      }
    return true;
  }
  return false;
}

bool ConfigurationFile::remove(const String &section, const String &name)
{
  if(configs.find(section) != configs.end())
  {
    ConfigSet &set = configs.at(section);
    
    if(set.remove(name))
    {
      if(set.isEmpty())
      {
        configs.erase(configs.find(section)); // Remove section
      }
      
      return true;
    }
  }
  return false;
}

bool ConfigurationFile::removeFile()
{
  if(_fileName.size() == 0)
  {
    logger(LOG_ERROR) << "ConfigurationFile: No file present for current configuration" << std::endl;
    return false;
  }
  
  InputFileStream f(_fileName);
  
  if(!f.good())
  {
    logger(LOG_ERROR) << "ConfigurationFile: Could not find file '" << _fileName << "'" << std::endl;
    return false;
  }
  
  f.close();
  
  if(::remove(_fileName.c_str()) != 0)
  {
    logger(LOG_ERROR) << "ConfigurationFile: Failed to remove file '" << _fileName << "'" << std::endl;
    return false;
  }
  
  return true;
}



bool MainConfigurationFile::read(const String &configFile)
{
  if(!ConfigurationFile::read(configFile))
    return false;
  
  _cameraProfiles.clear();
  _cameraProfileNames.clear();
  _currentCameraProfile = 0;
  _currentCameraProfileID = -1;
  
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

int MainConfigurationFile::addCameraProfile(const String &profileName, const int parentID)
{
  if(_cameraProfiles.find(parentID) == _cameraProfiles.end())
    return -1;
  
  int maxID = 0;
  for(auto c = _cameraProfileNames.begin(); c != _cameraProfileNames.end(); c++)
  {
    maxID = std::max(maxID, c->first);
    
    if(c->second == profileName)
      return -1;
  }
  
  int id = maxID + 1;
  _cameraProfiles[id] = ConfigurationFile(this);
  _cameraProfiles[id].setInteger("global", "id", id);
  _cameraProfiles[id].setInteger("global", "parent", parentID);
  _cameraProfiles[id].set("global", "name", profileName);
  
  _cameraProfileNames[id] = profileName;
  
  String fileName = profileName;
  fileName.erase(std::remove(fileName.begin(), fileName.end(), ' '), fileName.end());
  
  fileName = _mainConfigName + fileName + ".conf";
  
  if(!_cameraProfiles[id].write(fileName))
  {
    logger(LOG_WARNING) << "MainConfigurationFile: Could not write configuration file for '" << profileName << "' to '" << fileName << "'." << std::endl;
  }
  
  if(!_updateCameraProfileList())
  {
    logger(LOG_WARNING) << "MainConfigurationFile: Could not update profile list in main configuration file '" << _fileName << "'." << std::endl;
  }
  
  return id;
}

bool MainConfigurationFile::_updateCameraProfileList()
{
  String n;
  
  bool first = true;
  
  for(auto c = _cameraProfiles.begin(); c != _cameraProfiles.end(); c++)
  {
    if(!first)
      n += ", ";
    
    first = false;
    
    n += c->second.getConfigFileName();
  }
  
  if(!set("core", "camera_profiles", n))
    return false;
  
  if(!write())
    return false;
  
  return true;
}


bool MainConfigurationFile::removeCameraProfile(const int id)
{
  if(_cameraProfiles.find(id) == _cameraProfiles.end())
    return false;
  
  ConfigurationFile *c = &_cameraProfiles.at(id);
  
  if(!c->removeFile())
    return false;
  
  _cameraProfiles.erase(_cameraProfiles.find(id));
  
  if(_cameraProfileNames.find(id) != _cameraProfileNames.end())
    _cameraProfileNames.erase(_cameraProfileNames.find(id));
  
  if(!_updateCameraProfileList())
  {
    logger(LOG_WARNING) << "MainConfigurationFile: Could not update profile list in main configuration file '" << _fileName << "'." << std::endl;
  }
  
  return true;
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