/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_CONFIGURATION_H
#define VOXEL_CONFIGURATION_H

#include "Common.h"
#include "Serializable.h"
#include "DataPacket.h"

#define FILE_PREFIX "file:"

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

struct ConfigDataPacket: public DataPacket
{
  enum PacketType
  {
    PACKET_CONFIG = 0,
    PACKET_2D_DATA_FILE = 1
  };
  
  ConfigDataPacket(): DataPacket() {}
};

class MainConfigurationFile;

class VOXEL_EXPORT ConfigurationFile
{
protected:
  bool _get(const String &section, const String &name, String &value) const;
  bool _set(const String &section, const String &name, const String &value);
  
  String _fileName;
  
  Map<String, Vector<ByteType>> _dataFiles;
  
  MainConfigurationFile *_mainConfigurationFile;
  
  int _id, _parentID;
  String _profileName;
  
  mutable Mutex _mutex;
  
  bool _serializeAllDataFiles(OutputStream &out);
  
  template <typename T>
  bool _getData(const String &fileName, Vector<T> &data);
  
  template <typename T>
  bool _setData(const String &fileName, const Vector<T> &data);  
  
  bool _copyFromParentIfNotPresent(ConfigurationFile *other);

public:
  typedef Map<String, ConfigSet> ConfigSetMap;
  
  enum Location
  {
    IN_HOST = 0,
    IN_CAMERA = 1
  };
  
protected:
  Location _location;
  
public:
  ConfigSetMap configs;
  
  inline Location getLocation() const { return _location; }
  
  virtual bool isPresent(const String &section, const String &name, bool includeParent = true) const;
  virtual String get(const String &section, const String &name) const;
  
  inline void setID(const int id) { _id = id; setInteger("global", "id", id); }
  
  template <typename T>
  bool getFile(const String &section, const String &name, String &fileName, Vector<T> &data);
  
  int getInteger(const String &section, const String &name) const;
  float getFloat(const String &section, const String &name) const;
  bool getBoolean(const String &section, const String &name) const;
  
  virtual bool set(const String &section, const String &name, const String &value);
  
  template <typename T>
  bool setFile(const String &section, const String &name, const String &fileName, const Vector<T> &data);
  
  bool setInteger(const String &section, const String &name, int value);
  bool setFloat(const String &section, const String &name, float value);
  bool setBoolean(const String &section, const String &name, bool value);
  
  bool remove(const String &section, const String &name);
  
  virtual bool getConfigSet(const String &section, const ConfigSet *&configSet) const;
  
  virtual bool read(const String &configFile);
  virtual bool read(InputStream &in);
  
  virtual bool write(const String &configFile = "");
  virtual bool write(OutputStream &out);
  
  bool isValidCameraProfile();
  
  virtual bool removeFile();
  
  inline void clear() { Lock<Mutex> _(_mutex); configs.clear(); }
  
  inline const String &getConfigFileName() { return _fileName; }
  
  ConfigurationFile(): ConfigurationFile(0) {}
  ConfigurationFile(MainConfigurationFile *mainConfigurationFile): 
  _mainConfigurationFile(mainConfigurationFile), _id(-1), _parentID(-1), _location(IN_HOST) {}
  
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
    _location = other._location;
    _dataFiles = other._dataFiles;
    return *this;
  }
  
  inline bool mergeParentConfiguration()
  {
    return _copyFromParentIfNotPresent(this);
  }
  
  inline ConfigurationFile &copy(const ConfigurationFile &other)
  {
    return operator =(other);
  }
  
  virtual ~ConfigurationFile() {}
  
  friend class MainConfigurationFile;
};

template <typename T>
bool ConfigurationFile::getFile(const String &section, const String &name, String &fileName, Vector<T> &data)
{
  String v = get(section, name);
  
  if(v.compare(0, sizeof(FILE_PREFIX) - 1, FILE_PREFIX) != 0 || v.size() <= sizeof(FILE_PREFIX) - 1)
  {
    logger(LOG_ERROR) << "ConfigurationFile: section = " << section << ", name = " << name << ", is not a file type." << std::endl;
    return false;
  }
  
  fileName = v.substr(sizeof(FILE_PREFIX) - 1);
  
  return _getData(fileName, data);
}

template <typename T>
bool ConfigurationFile::setFile(const String &section, const String &name, const String &fileName, const Vector<T> &data)
{
  return _setData(fileName, data) && set(section, name, "file:" + fileName);
}

template <typename T>
bool ConfigurationFile::_getData(const String &fileName, Vector<T> &data)
{
  if(_dataFiles.find(fileName) == _dataFiles.end())
  {
    Configuration c;
    
    String f = fileName;
    if(!c.getConfFile(f))
    {
      logger(LOG_ERROR) << "ConfigurationFile: Could not locate file '" << fileName << "'" << std::endl;
      return false;
    }
    
    InputFileStream fs(f, std::ios::in | std::ios::binary | std::ios::ate);
    
    if(!fs.good())
    {
      logger(LOG_ERROR) << "ConfigurationFile: Could not open file '" << fileName << "'" << std::endl;
      return false;
    }
    
    int size = fs.tellg();
    
    if(size == 0)
    {
      logger(LOG_ERROR) << "ConfigurationFile: Null config data '" << f << "'" << std::endl;
      return false;
    }
    
    Vector<ByteType> &d = _dataFiles[fileName];
    
    d.resize(size);
    fs.seekg(std::ios::beg);
    fs.clear();
    fs.read((char *)d.data(), size);
  }
  
  Vector<ByteType> &d = _dataFiles[fileName];
  
  data.resize((d.size() + sizeof(T)/2)/sizeof(T));
  
  memcpy(data.data(), d.data(), d.size());
  
  return true;
}

template <typename T>
bool ConfigurationFile::_setData(const String &fileName, const Vector<T> &data)
{
  Configuration c;
  
  String f = fileName;
  if(!c.getConfFile(f))
  {
    logger(LOG_ERROR) << "ConfigurationFile: Could not locate file '" << fileName << "'" << std::endl;
    return false;
  }
  
  OutputFileStream fs(f, std::ios::out | std::ios::binary);
  
  if(!fs.good())
  {
    logger(LOG_ERROR) << "ConfigurationFile: Could not open file '" << fileName << "'" << std::endl;
    return false;
  }
  
  fs.write(data.data(), data.size()*sizeof(T));
  
  fs.close();
  
  _dataFiles[fileName].resize(data.size()*sizeof(T));
  
  memcpy(_dataFiles[fileName].data(), data.data(), data.size()*sizeof(T));
  
  return true;
}


class VOXEL_EXPORT MainConfigurationFile: public ConfigurationFile
{
protected:
  Map<int, ConfigurationFile> _cameraProfiles;
  Map<int, String> _cameraProfileNames;
  int _defaultCameraProfileID, _defaultCameraProfileIDInHardware;
  int _currentCameraProfileID;
  ConfigurationFile *_currentCameraProfile;

  bool _updateCameraProfileList();
  
  String _mainConfigName, _hardwareID;
  
  int _getNewCameraProfileID(bool inHost = true);
  
public:
  typedef Function<bool(SerializedObject &)> HardwareSerializer;
  
protected:
  HardwareSerializer _hardwareReader, _hardwareWriter;
    
public:
  MainConfigurationFile(const String &name, const String &hardwareID, HardwareSerializer hardwareReader = 0, HardwareSerializer hardwareWriter = 0): 
  _currentCameraProfile(nullptr), _defaultCameraProfileID(-1), _defaultCameraProfileIDInHardware(-1), _mainConfigName(name), _hardwareReader(hardwareReader), _hardwareWriter(hardwareWriter) {}
  
  virtual bool read(const String &configFile);
  
  bool readFromHardware();
  bool writeToHardware();
  
  inline void setHardwareReader(HardwareSerializer hardwareReader) { _hardwareReader = hardwareReader; }
  inline void setHardwareWriter(HardwareSerializer hardwareWriter) { _hardwareWriter = hardwareWriter; }
  
  virtual String get(const String &section, const String &name) const;
  virtual bool isPresent(const String &section, const String &name, bool includeParent = true) const;
  
  int addCameraProfile(const String &profileName, const int parentID);
  bool setCurrentCameraProfile(const int id);
  bool removeCameraProfile(const int id);
  bool saveCameraProfileToHardware(const int id);
  
  ConfigurationFile *getDefaultCameraProfile();
  ConfigurationFile *getCameraProfile(const int id);
  
  template <typename T>
  bool getFile(const String &section, const String &name, String &fileName, Vector<T> &data);
  
  int getDefaultCameraProfileID() 
  { 
    if(_defaultCameraProfileIDInHardware >= 0) 
      return _defaultCameraProfileIDInHardware;
    else
      return _defaultCameraProfileID; 
  }
  
  bool setDefaultCameraProfile(const int id);
  
  inline void setHardwareID(const String &hwID) { _hardwareID = hwID; }
  
  int getCurrentProfileID() { return _currentCameraProfileID; }
  
  bool getCameraProfileName(const int id, String &cameraProfileName);
  
  const Map<int, String> &getCameraProfileNames() { return _cameraProfileNames; }
 
  virtual ~MainConfigurationFile() {}
};

template <typename T>
bool MainConfigurationFile::getFile(const String &section, const String &name, String &fileName, Vector<T> &data)
{
  if(!_currentCameraProfile || !_currentCameraProfile->getFile(section, name, fileName, data))
    return ConfigurationFile::getFile(section, name, fileName, data);
  else
    return true;
}


}

#endif // CONFIGURATION_H