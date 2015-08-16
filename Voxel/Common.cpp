/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Common.h"
#include <sstream>
#include <iomanip>

#ifdef LINUX
#include <dirent.h>
#elif defined(WINDOWS)
#include <windows.h>
#endif

#ifdef LINUX
#define DIR_SEP '/'
#elif defined(WINDOWS)
#define DIR_SEP '\\'
#endif

#ifdef WINDOWS
#include <intrin.h>
#pragma intrinsic(_BitScanReverse)
#endif

#include <sys/types.h>
#include <sys/stat.h>

namespace Voxel
{

String getHex(uint16_t value)
{
  std::ostringstream s;
  s << std::hex << std::setw(4) << std::setfill('0') << value;
  return s.str();
}

void split(const String &str, const char delimiter, Vector<String> &split)
{
  split.clear();
  
  std::size_t pos = str.find(delimiter, 0);
  std::size_t previous = 0;
  
  while (pos != String::npos)
  {
    split.push_back(str.substr(previous, pos - previous));
    
    previous = pos + 1;
    pos = str.find(delimiter, previous);
  }
  
  split.push_back(str.substr(previous, pos));
}

void breakLines(const String &str, std::ostream &out, const uint maxPerLine, const String &newlinePrefix)
{
  String s = str;
  
  int breakPos;
  
  while(s.size())
  {
    if(s.size() <= maxPerLine)
    {
      out << s << "\n" << newlinePrefix;
      break;
    }
      
    breakPos = s.rfind(' ', maxPerLine);
    
    if(breakPos == String::npos)
      breakPos = maxPerLine;
    
    out << s.substr(0, breakPos + 1) << "\n" << newlinePrefix;
    s = s.substr(breakPos + 1);
  }
}


int getFiles(const String &dir, const String &matchString, Vector<String> &files)
{
  files.clear();

#ifdef LINUX
  DIR *dp;
  struct dirent *dirp;
  
  if((dp  = opendir(dir.c_str())) == NULL)
    return -1;
  
  while ((dirp = readdir(dp)) != NULL) 
  {
    if(dirp->d_type != DT_REG) // regular file or not?
      continue;
    
    String n = dirp->d_name;
    
    if(n.find(matchString) != String::npos)
    {
      files.push_back(dir + DIR_SEP + n);
    }
  }
  closedir(dp);
#elif defined(WINDOWS)
  WIN32_FIND_DATA ffd;

  String basePath;

  if (dir.size() == 0)
    basePath = "";
  else
    basePath = dir + DIR_SEP;

  HANDLE hFind = FindFirstFile((basePath + "*").c_str(), &ffd);

  if(INVALID_HANDLE_VALUE == hFind)
    return 0;
  
  // List all the files in the directory with some info about them.
  do
  {
    if(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      continue;

    String n = ffd.cFileName;

    if(n.find(matchString) != String::npos)
      files.push_back(basePath + n);

  } while (FindNextFile(hFind, &ffd) != 0);

  FindClose(hFind);
#endif

  return files.size();
}

//gcf function - return gcd of two numbers
uint gcd(uint n, uint m)
{
  uint gcd, remainder;
  
  while (n != 0)
  {
    remainder = m % n;
    m = n;
    n = remainder;
  }
  
  gcd = m;
  
  return gcd;
}

unsigned int nearestPowerOf2(unsigned int value, unsigned int &index)
{
  unsigned int result;
#ifdef LINUX
  index = (sizeof(value) * 8 - __builtin_clz(value));
  result = 1 << index;
#elif defined(WINDOWS)
  unsigned long i;
  if (_BitScanReverse(&i, value))
  {
    index = i + 1;
    result = 1 << index;
  }
  else
  {
    result = 1;
    index = 0;
  }
#endif

  if ((result >> 1) == value)
  {
    index -= 1;
    return value;
  }
  else
    return result;
}

String dirname(const String &filename)
{
  String s = filename;
  if (s.size() <= 1) //Make sure it's possible to check the last character.
  {
    return s;
  }
  if (*(s.rbegin() + 1) == DIR_SEP) //Remove trailing slash if it exists.
  {
    s.pop_back();
  }
  s.erase(std::find(s.rbegin(), s.rend(), DIR_SEP).base(), s.end());
  return s;
}


String basename(const String &filename)
{
  String s = filename;
  if (s.size() <= 1) //Make sure it's possible to check the last character.
  {
    return s;
  }
  if (*(s.rbegin() + 1) == DIR_SEP) //Remove trailing slash if it exists.
  {
    s.pop_back();
  }
  s.erase(s.begin(), std::find(s.rbegin(), s.rend(), DIR_SEP).base());
  return s;
}

bool isFilePresent(const String &filename)
{
  struct stat info;
  
  if(stat(filename.c_str(), &info) != 0)
    return false;
  else
    return true;
}


bool isDirectory(const String &filename)
{
  struct stat info;
  
  if(stat(filename.c_str(), &info ) != 0)
    return false;
  else if(info.st_mode & S_IFDIR)  // S_ISDIR() doesn't exist on my windows 
    return true;
  else
    return false;
}

bool makeDirectory(const String &filename)
{
#ifdef WINDOWS
  return CreateDirectory(filename.c_str(), NULL);
#elif defined(LINUX)
  return mkdir(filename.c_str(), 0755) == 0;
#endif
}


  
}