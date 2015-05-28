/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_POINT_H
#define VOXEL_POINT_H

#define _USE_MATH_DEFINES
#include <math.h>

#include <type_traits>

#include "Common.h"

namespace Voxel
{
/**
 * \addtogroup Frm
 * @{
 */


class Point
{
public:
  float x, y, z; // in meters
  
  Point(): x(0), y(0), z(0) {}
  Point(float x, float y): x(x), y(y), z(0) {}
  Point(float x, float y, float z): x(x), y(y), z(z) {}
  
  inline float angle(const Point &other) const;
  inline float norm() const;
  
  inline float dot(const Point &other) const;
  inline Point cross(const Point &other) const;
  
  inline Point &operator -();
  inline Point operator +(const Point &other) const;
  inline Point operator *(const Point &other) const;
  inline Point operator *(const float &other) const;
};

float Point::dot(const Point &other) const
{
  return (x*other.x + y*other.y + z*other.z);
}

Point Point::cross(const Point &other) const
{
  Point p;
  p.x = y*other.z - z*other.y;
  p.y = z*other.x - x*other.z;
  p.z = x*other.y - y*other.x;
  
  return p;
}


float Point::angle(const Point &other) const
{
  float n1 = norm(), n2 = other.norm();
  
  if(floatEquals(n1, 0) || floatEquals(n2, 0))
    return 0;
  
  return acos(dot(other)/n1/n2);
}

float Point::norm() const
{
  return sqrt(x*x + y*y + z*z);
}

inline Point &Point::operator -()
{
  x = -x;
  y = -y;
  z = -z;
  return *this;
}

Point Point::operator*(const Point &other) const
{
  return Point(x*other.x, y*other.y, z*other.z);
}

Point Point::operator*(const float &other) const
{
  return Point(x*other, y*other, z*other);
}

Point Point::operator +(const Point &other) const
{
  return Point(x + other.x, y + other.y, z + other.z);
}



class IntensityPoint: public Point
{
public:
  float i; // normalized 0-1
  
  static IntensityPoint *typeCast(Point *ptr)
  {
    return (IntensityPoint *)(ptr);
  }
};

class RGBPoint: public Point
{
public:
  float r, g, b; // normalized 0-1
  
  static RGBPoint *typeCast(Point *ptr)
  {
    return (RGBPoint *)(ptr);
  }
};

class Orientation // in radial co-ordinates
{
public:
  float theta, phi; // in radians
};
/**
 * @}
 */

}

#endif // VOXEL_POINT_H
