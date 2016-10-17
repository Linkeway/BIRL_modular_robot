/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML.h"

CML_NAMESPACE_START()

uint32 bytes_to_uint32( byte *b )
{
   uint32 ret;
   ret  = (uint32)(ByteCast(b[0]));
   ret |= (uint32)(ByteCast(b[1])) << 8;
   ret |= (uint32)(ByteCast(b[2])) << 16;
   ret |= (uint32)(ByteCast(b[3])) << 24;
   return ret;
}

uint16 bytes_to_uint16( byte *b )
{
   uint16 ret;
   ret  = (uint16)(ByteCast(b[0]));
   ret |= (uint16)(ByteCast(b[1])) << 8;
   return ret;
}

int32 bytes_to_int32( byte *b )
{
   return (int32)bytes_to_uint32(b);
}

int16 bytes_to_int16( byte *b )
{
   return (int16)bytes_to_uint16(b);
}

void int16_to_bytes( int16 i, byte *b )
{
   b[0] = ByteCast(i);
   b[1] = ByteCast(i>>8);
}

void uint16_to_bytes( uint16 i, byte *b )
{
   b[0] = ByteCast(i);
   b[1] = ByteCast(i>>8);
}

void int32_to_bytes( int32 i, byte *b )
{
   b[0] = ByteCast(i);
   b[1] = ByteCast(i>>8);
   b[2] = ByteCast(i>>16);
   b[3] = ByteCast(i>>24);
}

void uint32_to_bytes( uint32 i, byte *b )
{
   b[0] = ByteCast(i);
   b[1] = ByteCast(i>>8);
   b[2] = ByteCast(i>>16);
   b[3] = ByteCast(i>>24);
}

CML_NAMESPACE_END()

