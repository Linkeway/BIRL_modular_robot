/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file 
*/

#include <stdio.h>
#include "CML_Firmware.h"
#ifdef CML_FILE_ACCESS_OK

CML_NAMESPACE_USE();

/* local defines */
#define CFF_MAGIC    0x03281995

/* Firmware error objects */
CML_NEW_ERROR( FirmwareError, open,   "Unable to open firmware file" );
CML_NEW_ERROR( FirmwareError, read,   "Error reading from firmware file");
CML_NEW_ERROR( FirmwareError, format, "Firmware file formatting error");
CML_NEW_ERROR( FirmwareError, crc,    "Firmware file CRC mismatch");
CML_NEW_ERROR( FirmwareError, alloc,  "Unable to allocate data array");

/* local types */
class File
{
   FILE *fp;
   bool bigEndian;
public:
   File( void ){ fp = 0; }
   ~File(){ Close(); }
   const Error *Open( const char *name );
   const Error *ReadInt16( uint16 &d );
   const Error *ReadInt32( uint32 &d );
   const Error *Seek( uint32 pos );
   uint32 CRC32( void );
   void Close()
   {
      if( fp ) fclose(fp);
      fp = 0;
   }
   void setSwap( bool big )
   {
      bigEndian = big;
   }
};

/***************************************************************************/
/**
Default constructor for firmware object.  The file needs to be loaded 
before this object can be used.
*/
/***************************************************************************/
Firmware::Firmware( void )
{
   start = 0;
   length = 0;
   fileVersion = 0;
   ampType = 0;
   data = 0;
}

/***************************************************************************/
/**
Destructor for firmware object.  Deallocates the data array if one was 
allocated.
*/
/***************************************************************************/
Firmware::~Firmware( void )
{
   if( data )
      delete data;
}

/***************************************************************************/
/**
Load the firmware image from a file.
@param name Name of the file to load
@return An error object.
*/
/***************************************************************************/
const Error *Firmware::Load( const char *name )
{
   const Error *err;
   File f;

   uint32 magic, crc;

   // Open the file
   err = f.Open( name );
   f.setSwap(true);

   // Read the magic number
   if( !err ) err = f.ReadInt32( magic );
   if( !err ) err = f.ReadInt32( crc );
   if( err ) return err;

   if( magic != CFF_MAGIC )
      return &FirmwareError::format;

   if( crc != f.CRC32() )
      return &FirmwareError::crc;

   // Read the rest of the header
   err = f.Seek( 8 );
   if( !err ) err = f.ReadInt16( fileVersion );
   if( !err ) err = f.ReadInt16( ampType );
   if( !err ) err = f.Seek( 20 );
   if( !err ) err = f.ReadInt32( start );
   if( !err ) err = f.ReadInt32( length );
   if( !err ) err = f.Seek( 256 );
   if( err ) return err;

   // We don't support firmware files before type 1
   if( fileVersion < 1 )
      return &FirmwareError::format;

   // Check for a reasonable amplifier type:
   //   0 - Accelus (not supported)
   //   1 - Junus (not supported)
   //   2 - Accelnet / Xenus / Stepnet
   //   3 - Newer amp line using different DSP
   //   4 - I/O module
   if( ampType < 2 )
      return &FirmwareError::format;

   // Allocate an array to hold the amplifier data
   data = new uint16[ length ];
   if( !data ) return &FirmwareError::alloc;

   for( uint32 i=0; i<length; i++ )
   {
      err = f.ReadInt16( data[i] );
      if( err ) return err;
   }

   f.Close();
   return 0;
}

/***************************************************************************/
/**
Open the specified file.
@param name The file name/path
@return An error object
*/
/***************************************************************************/
const Error *File::Open( const char *name )
{
   fp = fopen( name, "rb" );
   if( !fp ) return &FirmwareError::open;
   return 0;
}

/***************************************************************************/
/**
Read a 16-bit integer from the file.  This function handles both big endian
and little endian file formats.
@param d  Data will be stored here
@return An error object
*/
/***************************************************************************/
const Error *File::ReadInt16( uint16 &d )
{
   uint8 data[2];
   if( fread( data, 1, 2, fp ) != 2 )
      return &FirmwareError::read;

   if( bigEndian )
      d = ((uint16)data[1]) | ((uint16)data[0]<<8);
   else
      d = ((uint16)data[0]) | ((uint16)data[1]<<8);

   return 0;
}

/***************************************************************************/
/**
Read a 32-bit integer from the file.  This function handles both big endian
and little endian file formats.
@param d  Data will be stored here
@return An error object
*/
/***************************************************************************/
const Error *File::ReadInt32( uint32 &d )
{
   uint8 data[4];
   if( fread( data, 1, 4, fp ) != 4 )
      return &FirmwareError::read;

   if( bigEndian )
   {
      d = ((uint32)data[3])     | ((uint32)data[2]<<8) |
	  ((uint32)data[1]<<16) | ((uint32)data[0]<<24);
   }
   else
   {
      d = ((uint32)data[0])     | ((uint32)data[1]<<8) |
	  ((uint32)data[2]<<16) | ((uint32)data[3]<<24);
   }

   return 0;
}

/***************************************************************************/
/**
Seek to the absolute byte position in the file.
@param pos The byte position to seek
@return An error object.
*/
/***************************************************************************/
const Error *File::Seek( uint32 pos )
{
   if( fseek( fp, pos, SEEK_SET ) == 0 )
      return 0;

   return &FirmwareError::read;
}

/***************************************************************************/
/**
Calculate the CRC of the file.  The CRC value is calculated starting
at the present file position, and running through the end of the file.
*/
/***************************************************************************/

#define CRC32_POLYNOMIAL   0xEDB88320
uint32 File::CRC32( void )
{
   uint32 crcTable[256];

   for( int i=0; i<256; i++ )
   {
      uint32 crc = i;
      for( int j=0; j<8; j++ )
      {
	 if( crc & 1)
	    crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
	 else
	    crc >>= 1;
      }
      crcTable[ i ] = crc;
   }

   uint32 crc = 0xFFFFFFFF;
   int ch;

   while( (ch=fgetc(fp)) != EOF )
   {
      uint32 temp1 = crc>>8;
      uint32 temp2 = crcTable[ (crc ^ ch) & 0xff ];
      crc = temp1 ^ temp2;
   }

   crc ^= 0xFFFFFFFF;
   return crc;
}
#endif
