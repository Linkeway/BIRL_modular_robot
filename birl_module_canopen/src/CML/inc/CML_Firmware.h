/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines classes related to the Copley amplifier
Firmware object.  The firmware object is used to update the
program within the amplifier.  An object of this type must
be passed to the Amp::FirmwareUpdate method to perform this task.

Note that firmware updates are likely to be rare, and are not
part of normal operation.

*/

#ifndef _DEF_INC_FIRMWARE
#define _DEF_INC_FIRMWARE

#include "CML_Settings.h"
#include "CML_Error.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This class represents error conditions that can occur while accessing
a Copley Controls amplifier firmware object.
*/
/***************************************************************************/
class FirmwareError: public Error
{
public:
   /// Unable to open specified firmware file
   static const FirmwareError open;

   /// Error reading from firmware file
   static const FirmwareError read;

   /// File format error
   static const FirmwareError format;

   /// File CRC error
   static const FirmwareError crc;

   /// Memory allocation error 
   static const FirmwareError alloc;

protected:
   /// Standard protected constructor
   FirmwareError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**
Copley Controls amplifier firmware object.  This object is used to represent
a firmware file which can be uploaded to an amplifier.

Note that uploading firmware to the amplifier is not part of normal operation.
The amplifier firmware is stored in internal Flash memory, and only needs to 
be updated if a new version with new features or bug fixes is produced by
Copley Controls Corporation.
*/
/***************************************************************************/
class Firmware
{
   /// Private copy constructor (not supported)
   Firmware( const Firmware & ){}

   /// Private assignment operator (not supported)
   Firmware &operator=( const Firmware & ){ return *this;}

private:
   /// Starting address of the firmware
   uint32 start;

   /// Length of the firmware data (words)
   uint32 length;

   /// Firmware file version number
   uint16 fileVersion;

   /// Amplifier type code for firmware.
   uint16 ampType;

   /// Points to an array of 16-bit binary data.
   /// This is the firmware which needs to be downloaded.
   uint16 *data;

public:
   Firmware( void );
   virtual ~Firmware( void );
   const Error *Load( const char *name );

   /// Returns the firmware file version number
   /// @return The firmware file version number
   uint16 getFileVersion(){ return fileVersion; }

   /// Returns the amplifier type for this firmware
   /// @return The amplifier type for this firmware
   uint16 getAmpType(){ return ampType; }

   /// Returns the firmware starting address
   /// @return The firmware starting address
   uint32 getStart(){ return start; }

   /// Returns the firmware length (in words)
   /// @return The firmware length (in words)
   uint32 getLength(){ return length; }

   /// Returns the firmware binary data
   /// @return The firmware binary data
   uint16 *getData( void ){ return data; }

   /// This virtual function is called repeatedly during an
   /// amplifier firmware update.  It can be overloaded to 
   /// display the progress of the download.  This version
   /// does nothing.
   /// @param addr The address currently being downloaded.
   virtual void progress( uint32 addr ){}
};

CML_NAMESPACE_END()

#endif

