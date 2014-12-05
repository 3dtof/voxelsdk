/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_POINT_H
#define VOXEL_POINT_H

namespace Voxel
{

class Point
{
public:
  float x, y, z; // in meters
};

class IntensityPoint: public Point
{
public:
  float i; // normalized 0-1
};

class RGBPoint: public Point
{
public:
  float r, g, b; // normalized 0-1
};

class Orientation // in radial co-ordinates
{
public:
  float theta, phi; // in radians
};

}

#endif // VOXEL_POINT_H
