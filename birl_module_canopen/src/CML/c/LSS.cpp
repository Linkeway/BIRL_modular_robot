/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML.h"

CML_NAMESPACE_USE();

/***************************************************************************/
/**
  Default constructor for the LSS object.
  @param co A reference to the CANopen network object over which this 
  protocol will run.
  */
/***************************************************************************/
LSS::LSS( CanOpen &co )
{
   recvCS = 0;
   timeout = 15;
   Init( co, 2020 );
   EnableReceiver();
}

/***************************************************************************/
/**
  Search the CANopen network for Copley amplifiers.  This function uses the
  CANopen Layer Setting Services (LSS) protocol to find amplifiers
  on the network.  All Copley amplifiers on the CANopen network can be
  identified using this protocol, even if they do not have a valid CANopen 
  node ID number configured.

  On return from this function, the passed array will have been filled with
  the serial numbers of all the amplifiers found.  These serial numbers may
  then be passed to LSS::SetAmpNodeID to assign a node ID number to the 
  amplifier.

  Note that firmware support for the LSS protocol was added starting with 
  version 4.04.  Any amplifier on the network with earlier firmware will not
  be discovered using this technique.

  @param max     The maximum number of amplifier serial numbers to be returned.
  @param serial  An array where the amplifier serial numbers will be returned.
  This array must be at least max elements long. 

  @return The number of amplifiers actually found.  This is not limited to the
  max parameter value
  */
/***************************************************************************/
int LSS::FindAmplifiers( int max, uint32 serial[] )
{
   const Error *err;

   // Switch all devices on the network to LSS configure mode
   err = Xmit( 4, 1 );

   // Pass the vendor and product codes to search
   if( !err ) err = Xmit( 70, 0xAB );
   if( !err ) err = Xmit( 71, 0 );

   // Search for any product revision number
   if( !err ) err = Xmit( 72, 0 );
   if( !err ) err = Xmit( 73, 0xFFFFFFFF );

   if( err ) return 0;

   tot = 0;
   this->max = max;
   this->serial = serial;

   return FindAmpSerial( 0, 0xFFFFFFFF );
}

/***************************************************************************/
/**
  Get the current CANopen node ID of the specified amplifier.
  @param serial The serial number of the amplifier to query.
  @param nodeID The node ID will be returned here.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::GetAmpNodeID( uint32 serial, byte &nodeID )
{
   // Select just the specified amplifier
   const Error *err = SelectAmp( serial );
   if( err ) return err;

   // Get the node ID
   while( !sem.Get(0) );
   recvCS = 94;
   err = Xmit( 94 );
   if( !err ) err = sem.Get( timeout );
   recvCS = 0;
   if( err ) return err;

   nodeID = recvData & 0xFF;
   return 0;
}

/***************************************************************************/
/**
  Set the CANopen node ID of the specified amplifier.
  @param serial The serial number of the amplifier to update.
  @param nodeID The CANopen node ID to assign to this amplifier.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *LSS::SetAmpNodeID( uint32 serial, byte nodeID )
{
   if( nodeID < 1 || nodeID > 127 )
      return &CanOpenError::BadNodeID;

   // Select just the specified amplifier
   const Error *err = SelectAmp( serial );
   if( err ) return err;

   // Set the node ID
   while( !sem.Get(0) );
   recvCS = 17;
   err = Xmit( 17, nodeID );
   if( !err ) err = sem.Get( timeout );
   recvCS = 0;
   if( err ) return err;

   if( recvData & 0xFF )
      return &CanOpenError::BadNodeID;

   // Deselect the amplifier which causes it to start using
   // the new node ID.
   return Xmit( 4, 0 );
}

/***************************************************************************/
/**
  Put the specified amplifier into LSS configure mode.  All other amplifiers
  on the network are switched into LSS operational mode.
  */
/***************************************************************************/
const Error *LSS::SelectAmp( uint32 serial )
{
   const Error *err;

   // Switch all devices on the network out of LSS config mode
   err = Xmit( 4, 0 );

   // Put the amplifier with the specified serial number into config mode.
   if( !err ) err = Xmit( 64, 0xAB );
   if( !err ) err = Xmit( 65, 0 );
   if( !err ) err = Xmit( 66, 0 );
   if( err ) return err;

   // Clear out the semaphore and transfer the serial number message.  This
   // should cause the selected amplifier to respond
   while( !sem.Get(0) );
   recvCS = 68;

   err = Xmit( 67, serial );
   if( !err ) err = sem.Get( timeout );

   recvCS = 0;

   return err;
}

/***************************************************************************/
/**
  This method is called by the main CAN network listener when a new LSS 
  response frame is received.
  */
/***************************************************************************/
int LSS::NewFrame( CanFrame &frame )
{
   // Ignore any messages when not expected
   if( !recvCS )
      return 1;

   // When an expected message is received, capture
   // the data and post a semaphore.
   else if( frame.data[0] == recvCS )
   {
      recvData = bytes_to_uint32( &frame.data[1] );
      sem.Put();
   }

   return 1;
}

/***************************************************************************/
/**
  Find the serial number of the first amplifier in the passed range.
  @return the serial number, or zero if no amp found.
  */
/***************************************************************************/
uint32 LSS::FindAmpSerial( uint32 low, uint32 high )
{
   // Clear out my semaphore
   while( !sem.Get(0) );

   // Send the serial number range
   recvCS = 79;
   if( Xmit( 74, low ) ) return 0;
   if( Xmit( 75, high ) ) return 0;

   // Wait for any amplifiers in this range to respond.
   const Error *err;
   int ct = 0;

   while( 1 )
   {
      err = sem.Get( timeout );
      if( err )
	 break;
      ct++;
   }

   recvCS = 0;
   if( !ct ) return 0;

   if( low == high )
   {
      if( tot < max )
	 serial[tot++] = low;
      return 1;
   }

   uint32 mid = low + (high-low)/2;

   return FindAmpSerial( low, mid ) + 
      FindAmpSerial( mid+1, high );
}

/***************************************************************************/
/**
  Transmit a LSS CAN frame 
  @param cs The command specifier for this frame.
  @param data The data passed with the frame.
  */
/***************************************************************************/
const Error *LSS::Xmit( byte cs, uint32 data )
{
   CanFrame frame;
   frame.id = 2021;
   frame.type = CAN_FRAME_DATA;
   frame.length = 8;
   frame.data[0] = cs;
   frame.data[1] = ByteCast(data);
   frame.data[2] = ByteCast(data>>8);
   frame.data[3] = ByteCast(data>>16);
   frame.data[4] = ByteCast(data>>24);
   frame.data[5] = 0;
   frame.data[6] = 0;
   frame.data[7] = 0;

   return co->Xmit( frame );
}

