/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "Common.h"
#include <sstream>
#include <iomanip>

#include <sys/time.h>
#include <dirent.h>

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
  int pos = str.find(delimiter, 0);
  int previous = 0;
  
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