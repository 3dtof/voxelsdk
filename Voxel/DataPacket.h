/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DATA_PACKET_H
#define VOXEL_DATA_PACKET_H

#include "Common.h"
#include "SerializedObject.h"

#include "Logger.h"

namespace Voxel
{
  
struct VOXEL_EXPORT DataPacket
{
  char magic[6]; // supposed to be VOXEL
  
  uint8_t type; // PacketType
  
  uint32_t size;
  
  SerializedObject object;
  
  DataPacket() { strcpy(magic, "VOXEL"); }

  bool readHeader(SerializedObject &in);
  bool readHeader(InputStream &in);
  
  bool read(SerializedObject &in);
  bool read(InputStream &in);
  
  bool write(SerializedObject &out);
  bool write(OutputStream &out);
  
  inline bool verifyMagic() { return magic[0] == 'V' && magic[1] == 'O' && magic[2] == 'X' && magic[3] == 'E' && magic[4] == 'L'; }
};

  
  
}

#endif