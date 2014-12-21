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
#define DIR_SEP "/"
#elif defined(WINDOWS)
#define DIR_SEP "\\"
#endif


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
  std::size_t pos = str.find(delimiter, 0);
  std::size_t previous = 0;
  
  while (pos != String::npos)
  {
    split.push_back(str.substr(previous, pos));
    
    previous = pos + 1;
    pos = str.find(delimiter, previous);
  }
  
  split.push_back(str.substr(previous, pos));
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
  HANDLE hFind = FindFirstFile((dir + DIR_SEP + "*").c_str(), &ffd);

  if(INVALID_HANDLE_VALUE == hFind)
    return 0;
  
  // List all the files in the directory with some info about them.
  do
  {
    if(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      continue;

    String n = ffd.cFileName;

    if(n.find(matchString) != String::npos)
      files.push_back(dir + DIR_SEP + n);

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

  
}