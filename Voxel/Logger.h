/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_LOGGER_H
#define VOXEL_LOGGER_H

#include <iostream>
#include <sstream>
#include <thread>
#include <string>

namespace Voxel
{
  
enum LogLevel
{
  ERROR, 
  WARNING, 
  INFO, 
  DEBUG, 
  DEBUG1, 
  DEBUG2, 
  DEBUG3, 
  DEBUG4
};
  
class Logger
{
protected:
  std::ostream &_out = std::cerr;

  LogLevel _logLevel, // Allow log statements equal to or below _logLevel
  _currentLogLevel; // This holds log level for current statements
  
  const std::string _logLevelNames[8] = {
    "ERROR",
    "WARNING",
    "INFO",
    "DEBUG",
    "DEBUG1",
    "DEBUG2",
    "DEBUG3",
    "DEBUG4"
  };
  
public:
  Logger(LogLevel loglevel = ERROR): _logLevel(loglevel), _currentLogLevel(loglevel) {}
  
  Logger &operator()(LogLevel loglevel)
  {
    _currentLogLevel = loglevel;
    return *this << _logLevelNames[loglevel] << ": ";
  }
  
  void setDefaultLogLevel(LogLevel loglevel)
  {
    _logLevel = loglevel;
  }
  
  std::ostream &getStream()
  {
    return _out;
  }
  
  template <typename T>
  Logger &operator <<(const T &value)
  {
    if(_currentLogLevel <= _logLevel)
      _out << value;
    return *this;
  }
  
  typedef Logger &(*LoggerManipulator)(Logger &);
  
  typedef std::ostream &(*OStreamManipulator)(std::ostream &);
  
  Logger &operator <<(LoggerManipulator manip)
  {
    if(_currentLogLevel <= _logLevel)
      return (*manip)(*this);
    else
      return *this;
  }
  
  Logger &operator <<(OStreamManipulator manip)
  {
    if(_currentLogLevel <= _logLevel)
      (*manip)(_out);
    return *this;
  }
  
  virtual ~Logger()
  {
  }
};

extern Logger log;

Logger &endl(Logger &l);

}
  
#endif // VOXEL_LOGGER_H