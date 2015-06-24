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

/**
 * \ingroup Util
 */

class VOXEL_EXPORT Configuration
{
protected:
  struct _Path
  {
    String standardPath;
    String environmentVariable;
  };
  
  static const Map<String, _Path> _pathTypes;
  
  bool _getLocalPath(const String &type, String &path);
  bool _getPaths(const String &type, Vector<String> &paths);
  
  bool _get(const String &type, String &name);

  static bool _addPath(const String &type, const String &path);
  
public:
  inline bool getFirmwarePaths(Vector<String> &paths) { return _getPaths("fw", paths); }
  inline bool getConfPaths(Vector<String> &paths) { return _getPaths("conf", paths); }
  inline bool getLibPaths(Vector<String> &paths) { return _getPaths("lib", paths); }

  inline static bool addFirmwarePath(const String &path) { return _addPath("firmware", path); }
  inline static bool addConfPath(const String &path) { return _addPath("conf", path); }
  inline static bool addLibPath(const String &path) { return _addPath("lib", path); }
  
  inline bool getLocalFirmwarePath(String &path) { return _getLocalPath("fw", path); }
  inline bool getLocalConfPath(String &path) { return _getLocalPath("conf", path); }
  inline bool getLocalLibPath(String &path) { return _getLocalPath("lib", path); }
  
  /// Updates "name" to full path
  inline bool getConfFile(String &name) { return _get("conf", name); }
  inline bool getFirmwareFile(String &name) { return _get("firmware", name); }
  inline bool geLibFile(String &name) { return _get("lib", name); }
  
};

class ConfigurationFile;

class VOXEL_EXPORT ConfigSet
{
protected:
  bool _get(const String &name, String &value) const;
  bool _set(const String &name, const String &value);
  
public:
  Map<String, String> params;
  Vector<String> paramNames;
  
  bool remove(const String &name);
  
  bool isEmpty() { return params.size() == 0; }
  
  bool isPresent(const String &name) const;
  String get(const String &name) const;
  int getInteger(const String &name) const;
  float getFloat(const String &name) const;
  bool getBoolean(const String &name) const;
  
  bool set(const String &name, const String &value);
  bool setInteger(const String &name, int value);
  bool setFloat(const String &name, float value);
  bool setBoolean(const String &name, bool value);
  
  friend class ConfigurationFile;
};

class MainConfigurationFile;

class VOXEL_EXPORT ConfigurationFile
{
protected:
  bool _get(const String &section, const String &name, String &value) const;
  bool _set(const String &section, const String &name, const String &value);
  
  String _fileName;
  
  MainConfigurationFile *_mainConfigurationFile;
  
  int _id, _parentID;
  String _profileName;
  
  mutable Mutex _mutex;
  
public:
  typedef Map<String, ConfigSet> ConfigSetMap;
  
  ConfigSetMap configs;
  
  virtual bool isPresent(const String &section, const String &name) const;
  virtual String get(const String &section, const String &name) const;
  int getInteger(const String &section, const String &name) const;
  float getFloat(const String &section, const String &name) const;
  bool getBoolean(const String &section, const String &name) const;
  
  virtual bool set(const String &section, const String &name, const String &value);
  bool setInteger(const String &section, const String &name, int value);
  bool setFloat(const String &section, const String &name, float value);
  bool setBoolean(const String &section, const String &name, bool value);
  
  bool remove(const String &section, const String &name);
  
  virtual bool getConfigSet(const String &section, const ConfigSet *&configSet) const;
  
  virtual bool read(const String &configFile);
  virtual bool read(InputStream &in);
  
  virtual bool write(const String &configFile = "");
  virtual bool write(OutputStream &out);
  
  virtual bool removeFile();
  
  inline void clear() { Lock<Mutex> _(_mutex); configs.clear(); }
  
  inline const String &getConfigFileName() { return _fileName; }
  
  ConfigurationFile(): ConfigurationFile(0) {}
  ConfigurationFile(MainConfigurationFile *mainConfigurationFile): 
  _mainConfigurationFile(mainConfigurationFile), _id(-1), _parentID(-1) {}
  
  ConfigurationFile(const ConfigurationFile &other)
  {
    operator =(other);
  }
  
  inline ConfigurationFile &operator =(const ConfigurationFile &other) 
  { 
    configs = other.configs; 
    _fileName = other._fileName;
    _mainConfigurationFile = other._mainConfigurationFile;
    _id = other._id;
    _profileName = other._profileName;
    _parentID = other._parentID;
    return *this;
  }
  
  virtual ~ConfigurationFile() {}
  
  friend class MainConfigurationFile;
};

class VOXEL_EXPORT MainConfigurationFile: public ConfigurationFile
{
  Map<int, ConfigurationFile> _cameraProfiles;
  Map<int, String> _cameraProfileNames;
  int _defaultCameraProfileID;
  int _currentCameraProfileID;
  ConfigurationFile *_currentCameraProfile;

  bool _updateCameraProfileList();
  
  String _mainConfigName;
public:
  MainConfigurationFile(const String &name): _currentCameraProfile(0), _mainConfigName(name) {}
  
  virtual bool read(const String &configFile);
  
  virtual String get(const String &section, const String &name) const;
  virtual bool isPresent(const String &section, const String &name) const;
  
  int addCameraProfile(const String &profileName, const int parentID);
  bool setCurrentCameraProfile(const int id);
  bool removeCameraProfile(const int id);
  
  ConfigurationFile *getDefaultCameraProfile();
  ConfigurationFile *getCameraProfile(const int id);
  
  int getDefaultCameraProfileID() { return _defaultCameraProfileID; }
  int getCurrentProfileID() { return _currentCameraProfileID; }
  
  bool getCameraProfileName(const int id, String &cameraProfileName);
  
  const Map<int, String> &getCameraProfileNames() { return _cameraProfileNames; }
  
  virtual ~MainConfigurationFile() {}
};


}

#endif // CONFIGURATION_H
