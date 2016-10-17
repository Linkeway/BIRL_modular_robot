/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file contains the base classes used to define the low level interface
to the CAN network hardware.  
*/

#ifndef _DEF_INC_CAN_INTF
#define _DEF_INC_CAN_INTF

#include "CML_Error.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This enumeration is used to identify the type of CAN frame.
*/
/***************************************************************************/
enum CAN_FRAME_TYPE
{
   /// Standard CAN data frame
   CAN_FRAME_DATA,

   /// Remote frame
   CAN_FRAME_REMOTE,

   /// Error frame
   CAN_FRAME_ERROR
};

/**
Low level CAN data frame.
This class is used to represent the basic frame of information that is
passed over the CAN network.

A frame of CAN data consists of a message ID value (either 11 or 29 bits
depending on whether standard or extended frames are in use), 0 to 8 bytes
of data, and some special attributes.

Frame objects are passed to the CanInterface::Xmit() function, and
filled in by the CanInterface::Recv() function.
*/
struct CanFrame
{
   /// Identifies the frame type.  
   CAN_FRAME_TYPE type;

   /// The CAN message ID.
   /// If bit 29 is clear, this is a standard 11 bit ID (bits 0-10 hold the ID).
   /// If bit 29 is set, this is an extended 29 bit ID (bits 0-28 hold the ID).
   /// Bits 30 and 31 are not presently used.
   int32 id;

   /// Gives the number of bytes of data included in the frame.
   /// The length of a frame may range from 0 to 8 bytes
   byte length;

   /// Holds any data sent with the frame.  A frame may be
   /// accompanied by 0 to 8 bytes of data.
   byte data[8];
};

/** 
Class used to represent an error condition returned 
from a CAN interface function.
*/
class CanError: public Error
{
public:
   /// Indicates that the specified port name is invalid
   static const CanError BadPortName;
   /// Indicates that the CAN port is not open.
   static const CanError NotOpen;
   /// Indicates an illegal attempt to open an already open port
   static const CanError AlreadyOpen;
   /// A parameter passed to the CAN member function is not valid.
   static const CanError BadParam;
   /// Generic error from the CAN driver.
   static const CanError Driver;
   /// Illegal baud rate specified
   static const CanError BadBaud;
   /// Timeout waiting on read/write
   static const CanError Timeout;
   /// CAN buffer overflow
   static const CanError Overflow;
   /// CAN bus is in the OFF state
   static const CanError BusOff;
   /// Indicates an invalid CAN ID passed
   static const CanError InvalidID;
   /// Unknown CAN error condition
   static const CanError Unknown;
   /// Unable to open CAN driver, or missing dll file
   static const CanError NoDriver;
   /// CAN driver memory allocation error
   static const CanError Alloc;
   /// Permission error opening CAN port
   static const CanError Permission;

protected:
   CanError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**
Abstract class used for low level interaction with CAN hardware.

This class contains methods that are used to open and
close the CAN network adapter, as well as transmit and
receive frames over the CAN network.  

The base CanInterface class defines a standard minimal
interface to the CAN network.  This base class does not
actually provide support for any hardware, rather
it should be extended by a class that provides access
to the actual CAN network adapter being used.
*/
/***************************************************************************/
class CanInterface
{
   /// Private copy constructor (not supported)
   CanInterface( const CanInterface & );

   /// Private assignment operator (not supported)
   CanInterface &operator=( const CanInterface & );
public:

   /***************************************************************************/
   /**
     Default constructor for a CAN interface object.
     */
   /***************************************************************************/
   CanInterface( void ): portName(0), canOpenPtr(0) {}

   /***************************************************************************/
   /**
     Standard constructor for the CAN interface object.
     The only thing that the constructor does is initialize the
     portName member variable to the value passed.
     @param port A string that may be used to identify which port to open.
     */
   /***************************************************************************/
   CanInterface( const char *port ): portName(port), canOpenPtr(0) {}

   virtual ~CanInterface();

   /// Set the name of the port.
   /// @param name The port name.
   virtual const Error *SetName( const char *name )
   {
      portName = name;
      return 0;
   }

   /**
     Open the CAN interface.
     @return A valid CAN error object
     */
   virtual const Error *Open( void ){ return 0; }

   /**
     Close the CAN interface.
     @return A valid CAN error object
     */
   virtual const Error *Close( void ){ return 0; }

   /**
     Set the CAN interface baud rate
     @param baud In bits / second
     @return A valid CAN error object
     */
   virtual const Error *SetBaud( int32 baud ){ return 0; }

   const Error *Recv( CanFrame &frame, int32 timeout=-1 );
   const Error *Xmit( CanFrame &frame, int32 timeout=0 );

   /***************************************************************************/
   /**
     Check an ID to make sure it's valid.  To be valid, a message ID
     must either be an 11 bit standard ID, or a 28-bit extended ID.  By
     convention, all extended ID's must have bit 29 set to identify them
     as such.
     @param id The ID to be checked
     @return A pointer to an error object, or NULL on success.
     */
   /***************************************************************************/
   static const Error *ChkID( int32 id )
   {
      if( !(id&0xFFFFF800) )
	 return 0;

      if( (id & 0xE0000000) != 0x20000000 )
	 return &CanError::InvalidID;

      return 0;
   }

protected:
   /// This string is initialized by the default constructor.
   /// It may be used to identify which CAN port to open, etc.
   const char *portName;

   /***************************************************************************/
   /**
     Receive the next CAN frame.  This is called by the public Recv function, and
     must be implemented by the actual CanInterace class.  It handles the hardware
     specific details of reading a message from the network.

     @param frame A reference to the frame object that will be filled by the read.
     @param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
     return immediately if no data is available.  A timeout of < 0 will 
     wait forever.
     @return A pointer to an error object, or NULL on success.
     */
   /***************************************************************************/
   virtual const Error *RecvFrame( CanFrame &frame, int32 timeout );

   /***************************************************************************/
   /**
     Write a CAN frame to the CAN network.  This is called by the public Xmit 
     function, and must be implemented by the actual CanInterface class.  It
     handles the hardware specific details of writing a message to the network.

     @param frame A reference to the frame to write.
     @param timeout The time to wait for the frame to be successfully sent.
     If the timeout is 0, the frame is written to the output queue and
     the function returns without waiting for it to be sent.  If the 
     timeout is <0 then the function will delay forever.
     @return A pointer to an error object, or NULL on success.
     */
   /***************************************************************************/
   virtual const Error *XmitFrame( CanFrame &frame, int32 timeout );

   int FindPortNumber( const char *id );

private:
   class CanOpen *canOpenPtr;
   friend class CanOpen;
};

CML_NAMESPACE_END()

#endif

