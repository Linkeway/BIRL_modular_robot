/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/**
\file
This file holds the code needed to implement CANopen Process Data Objects (PDOs).
*/

#include "CML.h"

CML_NAMESPACE_USE();

/**************************************************
* PDO Error objects
**************************************************/
CML_NEW_ERROR( PDO_Error, MapFull,       "The PDO Map is already full" );
CML_NEW_ERROR( PDO_Error, BitOverflow,   "Variable too long to add to map" );
CML_NEW_ERROR( PDO_Error, BitSizeError,  "The specified variable bit size is not supported" );

/***************************************************************************/
/**
Send a Remote Transmit Request (RTR) frame for this PDO.  The nodes that
transmit the PDO should transmit in response.
*/
/***************************************************************************/
const Error *TPDO::Request( void )
{
   CanFrame frame;

   frame.id = getRecvID();
   frame.type = CAN_FRAME_REMOTE;
   frame.length = (bitCt+7)>>3;

   return co->Xmit( frame );
}

/***************************************************************************/
/**
Add the passed variable to the end of the variable map associated with this
PDO.
@param var The variable to be added.
@return An error object.
*/
/***************************************************************************/
const Error *PDO::AddVar( Pmap &var )
{
   // Make sure I'm not already full
   if( mapCt == PDO_MAP_LEN )
      return &PDO_Error::MapFull;

   byte bits = var.GetBits();

   // Keep the variable size <= 64 bits
   if( bitCt + bits > 64 )
      return &PDO_Error::BitOverflow;

   // For now, my variables must be a multiple
   // of 8 bits long.
   if( bits & 0x07 )
      return &PDO_Error::BitSizeError;

   // Looks good, map it.
   map[mapCt++] = &var;
   bitCt += bits;
   return 0;
}

/***************************************************************************/
/**
New frame handler.  Split the data passed in the frame up among the variables
in my map.  After all variables have been updated, the virtual function 
Received() is called.  This can be used in sub-classes to indicate new data 
is available.
*/
/***************************************************************************/
int TPDO::NewFrame( CanFrame &frame )
{
   if( frame.type != CAN_FRAME_DATA )
      return 0;

   unsigned char *cptr = (unsigned char *)frame.data;

   for( int i=0; i<mapCt; i++ )
   {
      map[i]->Set( cptr );
      cptr += (map[i]->GetBits()>>3);
   }

   Received();
   return 0;
}

/***************************************************************************/
/**
Transmit the RPDO.
*/
/***************************************************************************/
const Error *RPDO::Transmit( CanOpen &co )
{
   CanFrame frame;

   // Initialize the CAN frame
   frame.id = id;
   frame.type = CAN_FRAME_DATA;
   frame.length = (bitCt+7)>>3;

   // Load the data from the mapping objects
   byte *cptr = frame.data;
   for( int i=0; i<mapCt; i++ )
   {
      map[i]->Get( cptr );
      cptr += (map[i]->GetBits()>>3);
   }

   // Transmit the frame
   return co.Xmit( frame );
}

