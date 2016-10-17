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
This file handles the initialization of the static variables (error codes)
used by the CanError and CanInterface classes.
*/

#include "CML.h"

CML_NAMESPACE_USE();

CML_NEW_ERROR( CanError, BadPortName,  "The CAN port name specified is unknown." );
CML_NEW_ERROR( CanError, NotOpen,      "The CAN interface is not open." );
CML_NEW_ERROR( CanError, AlreadyOpen,  "Attempt to open an interface that's already open." );
CML_NEW_ERROR( CanError, BadParam,     "One of the passed parameters is unacceptable." );
CML_NEW_ERROR( CanError, Driver,       "Generic CAN driver level error." );
CML_NEW_ERROR( CanError, BadBaud,      "Illegal CAN baud rate specified." );
CML_NEW_ERROR( CanError, Timeout,      "Timeout waiting on read/write." );
CML_NEW_ERROR( CanError, Overflow,     "CAN bus data overflow." );
CML_NEW_ERROR( CanError, BusOff,       "CAN bus off." );
CML_NEW_ERROR( CanError, InvalidID,    "The CAN ID is not valid." );
CML_NEW_ERROR( CanError, Unknown,      "Unknown / unspecified CAN error." );
CML_NEW_ERROR( CanError, NoDriver,     "Unable to open CAN driver, or missing dll file" );
CML_NEW_ERROR( CanError, Alloc,        "CAN driver memory allocation error" );
CML_NEW_ERROR( CanError, Permission,   "Permission error opening CAN port" );

/***************************************************************************/
/**
Receive the next CAN frame.  This is the public function used to read CAN
messages from the network.

@param frame A reference to the frame object that will be filled by the read.
@param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
		 return immediately if no data is available.  A timeout of < 0 will 
		 wait forever.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanInterface::Recv( CanFrame &frame, int32 timeout )
{
   const Error *err = RecvFrame( frame, timeout );
   if( !err ) 
      cml.LogCAN( true, frame );
   return err;
}

/***************************************************************************/
/**
Write a CAN frame to the CAN network.
@param frame A reference to the frame to write.
@param timeout The time to wait for the frame to be successfully sent.
		 If the timeout is 0, the frame is written to the output queue and
		 the function returns without waiting for it to be sent.  If the 
		 timeout is <0 then the function will delay forever.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanInterface::Xmit( CanFrame &frame, int32 timeout )
{
   cml.LogCAN( false, frame );
   return XmitFrame( frame, timeout );
}

/***************************************************************************/
/**
Standard destructor for base CanInterface object.
*/
/***************************************************************************/
CanInterface::~CanInterface()
{
   // Remove the link to this object from the CanOpen class.  This
   // prevents the CanOpen destructor from trying to close me.
   if( canOpenPtr )
   {
      canOpenPtr->stop();
      canOpenPtr->can = 0;
   }
}

/**************************************************
* These default implementations just delay the calling thread
* and return.
**************************************************/
const Error *CanInterface::RecvFrame( CanFrame &frame, int32 timeout )
{
   Thread::sleep( timeout );
   return &ThreadError::Timeout;
}

const Error *CanInterface::XmitFrame( CanFrame &frame, int32 timeout )
{
   Thread::sleep( timeout );
   return &ThreadError::Timeout;
}

/*************************************************************
 * This is a private utility function used by some of the 
 * CAN Interface objects.  It converts the port name string
 * to a port number using the following algorithm:
 *
 *  If the port name is NULL, then the port number is 0
 *  If the port name is of the form "<id string><x>" where
 *    <id string> is the passed string and <x> is an integer
 *    then the port number is <x>
 *  Otherwise, the port name doesn't match.
 *
 *  @param id The ID string to match the port name against
 *  @return The port number, or -1 on no match.
 ************************************************************/
int CanInterface::FindPortNumber( const char *id )
{
   if( portName == 0 ) 
      return 0; 

   int i;
   for( i=0; id[i]; i++ )
   {
      char ch = portName[i];
      if( ch >= 'a' && ch <= 'z' )
	 ch += 'A' - 'a';

      if( id[i] != ch )
	 return -1;
   }

   int num = 0;
   while( portName[i] )
   {
      char ch = portName[i];
      if( ch < '0' || ch > '9' )
	 return -1;
      num *= 10;
      num += ch - '0';
      i++;
   }
   return num;
}

