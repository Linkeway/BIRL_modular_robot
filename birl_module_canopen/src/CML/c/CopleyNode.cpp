/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file holds code to implement the CopleyNode object.
*/

#include "CML.h"

// local defines
#define MAX_MSG_BYTES   200        // Max number of data bytes / serial command

CML_NAMESPACE_USE();

// Node error objects
CML_NEW_ERROR( CopleyNodeError, SerialMsgLen, "Too much data passed for a serial port command" );
CML_NEW_ERROR( CopleyNodeError, SerialError,  "The device return a serial port error code"     );

/***************************************************************************/
/**
Send a serial port message to a Copley device over the CANopen network.

Copley devices use serial ports for some basic configuration purposes.  Most
of the functions available over the serial interface are also available in 
the CANopen object dictionary, but not everything.

This function allows a native serial port command to be sent over the CANopen
network.  It allows any remaining features of the device to be accessed when
only a CANopen connection is available.

@param opcode The command code to be sent to the amplifier.
@param ct     The number of 16-bit words of data to be sent to the amplifier.  On
              return, this parameter will hold the number of 16-bit words of response
	      data passed back from the amplifier.
@param max    The maximum number of words of response data that the data array can 
              hold.
@param data   An array of data to be passed to the node with the command.  On return,
              any response data (up to max words) will be passed here.  
	      If this parameter is not specified, then no data will be passed or returned
	      regardless of the values passed in max and ct.

@return       An error object, or null on success.
*/
/***************************************************************************/
const Error *CopleyNode::SerialCmd( uint8 opcode, uint8 &ct, uint8 max, uint16 *data )
{
   if( !data ) max = ct = 0;

   if( ct > MAX_MSG_BYTES/2 )
      return &CopleyNodeError::SerialMsgLen;

   uint8 buff[ MAX_MSG_BYTES + 1 ];
   buff[0] = opcode;

   int i;
   for( i=0; i<ct; i++ )
   {
      buff[2*i+1] = (uint8)data[i];
      buff[2*i+2] = (uint8)(data[i]>>8);
   }
	
   const CML::Error *err = sdo.Download( 0x2000, 0, 2*ct+1, buff );
   if( err ) return err;

   int32 len = MAX_MSG_BYTES+1;
   err = sdo.Upload( 0x2000, 0, len, (uint8 *)buff );
   if( err ) return err;

   lastSerialError = buff[0];
   if( lastSerialError )
      return &CopleyNodeError::SerialError;

   ct = (len-1)/2;
   if( ct > max ) ct = max;

   for( i=0; i<ct; i++ )
      data[i] = bytes_to_uint16( &buff[2*i+1] );

   return 0;
}

