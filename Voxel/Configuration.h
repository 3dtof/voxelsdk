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
  
  bool _getPaths(const String &type, Vector<String> &paths);
  
  bool _get(const String &type, String &name);

  static bool _addPath(const String &type, const String &path);
  
public:
  inline bool getFirmwarePaths(Vector<String> &paths) { return _getPaths("firmware", paths); }
  inline bool getConfPaths(Vector<String> &paths) { return _getPaths("conf", paths); }
  inline bool getLibPaths(Vector<String> &paths) { return _getPaths("lib", paths); }

  inline static bool addFirmwarePath(const String &path) { return _addPath("firmware", path); }
  inline static bool addConfPath(const String &path) { return _addPath("conf", path); }
  inline static bool addLibPath(const String &path) { return _addPath("lib", path); }
  
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
  
public:
  Map<String, String> params;
  Vector<String> paramNames;
  
  bool isPresent(const String &name) const;
  String get(const String &name) const;
  int getInteger(const String &name) const;
  float getFloat(const String &name) const;
  bool getBoolean(const String &name) const;
  
  friend class ConfigurationFile;
};

class MainConfigurationFile;

class VOXEL_EXPORT ConfigurationFile
{
protected:
  bool _get(const String &section, const String &name, String &value) const;
  
  String _fileName;
  
  MainConfigurationFile *_mainConfigurationFile;
  
public:
  typedef Map<String, ConfigSet> ConfigSetMap;
  
  ConfigSetMap configs;
  
  virtual bool isPresent(const String &section, const String &name) const;
  virtual String get(const String &section, const String &name) const;
  int getInteger(const String &section, const String &name) const;
  float getFloat(const String &section, const String &name) const;
  bool getBoolean(const String &section, const String &name) const;
  virtual bool getConfigSet(const String &section, const ConfigSet *&configSet) const;
  
  virtual bool read(const String &configFile);
  
  inline const String &getConfigFileName() { return _fileName; }
  
  ConfigurationFile(MainConfigurationFile *mainConfigurationFile = 0): _mainConfigurationFile(mainConfigurationFile) {}
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
public:
  MainConfigurationFile(): _currentCameraProfile(0) {}
  
  virtual bool read(const String &configFile);
  
  virtual String get(const String &section, const String &name) const;
  virtual bool isPresent(const String &section, const String &name) const;
  
  bool setCurrentCameraProfile(const int id);
  
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
