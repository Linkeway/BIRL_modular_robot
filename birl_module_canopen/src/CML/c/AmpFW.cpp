/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file contains code used to update an amplifier's firmware over the
CANopen network.

The firmware update protocol is a non-standard protocol which uses CAN
messages with the same IDs as the node's CANopen SDO protocol.  These 
message IDs were used because they are fixed for a given node ID.
*/

#include "CML_Copley.h"
#include "CML_AmpDef.h"

CML_NAMESPACE_USE();

/***************************************************************************/
/**
This class uses a proprietory protocol to update the amplifier's internal 
firmware.  This protocol uses the SDO transmit and receive CAN message ID
values.
*/
/***************************************************************************/
class FirmwareUpdater: public Receiver
{
   Semaphore sem;
   uint32 xmitId;

public:
   /// Response code from last received message
   uint16 resp;

   /// 32-bit data value received in last response.
   uint32 data;

   /***************************************************************************/
   /**
     Initialize the firmware updater using the ID parameters currently associated 
     with the SDO.

     @param sdo The SDO who's message ID values will be used for this updater.
     */
   /***************************************************************************/
   FirmwareUpdater( SDO &sdo )
   {
      xmitId = sdo.getXmitID();
      Init( sdo.GetCanOpen(), sdo.getRecvID() );
   }

   /***************************************************************************/
   /**
     Receive frame handler called by the receive task when a new CAN message of 
     the proper ID is received.

     This function simply copies the data out of the received frame and posts to 
     the semaphore associated with this object.

     @param frame The received CAN frame
     @return Always returns 1.
     */
   /***************************************************************************/
   int NewFrame( CanFrame &frame )
   {
      resp = ((uint16)frame.data[0]) | ((uint16)frame.data[1]<<8);

      data = ((uint32)frame.data[2])     | ((uint32)frame.data[3]<<8)  |
	     ((uint32)frame.data[4]<<16) | ((uint32)frame.data[5]<<24);

      sem.Put();
      return 1;
   }

   /***************************************************************************/
   /**
     Send a command with no parameters
     @param cmd The command code.  This code is part of the amplifiers
     special firmware upload protocol.
     @param timeout The time to wait for a response to this command.
     (milliseconds).
     @return An error object
     */
   /***************************************************************************/
   const Error *SendCmd( uint16 cmd, int32 timeout )
   {
      return SendCmd( cmd, 0, 0, timeout );
   }

   /***************************************************************************/
   /**
     Send a command to the amplifier.  The command is part of the amps special 
     firmware upload protocol documented elsewhere.
     @param cmd The command code to send
     @param ct The length of the parameter array passed
     @param param An array of additional parameter data passed with the command.
     @param timeout The time to wait for a response to this command.
     (milliseconds).
     @return An error object
     */
   /***************************************************************************/
   const Error *SendCmd( uint16 cmd, uint16 ct, uint16 *param, int32 timeout )
   {
      const Error *err;
      CanFrame frame;

      frame.id = xmitId;
      frame.type = CAN_FRAME_DATA;
      frame.length = 2 + 2*ct;

      frame.data[0] = ByteCast(cmd);
      frame.data[1] = ByteCast(cmd>>8);

      for( uint16 i=0; i<ct; i++ )
      {
	 frame.data[2*i+2] = ByteCast(param[i]);
	 frame.data[2*i+3] = ByteCast(param[i]>>8);
      }

      err = co->Xmit( frame );

      if( !err ) err = sem.Get(timeout);

      return err;
   }
};

/***************************************************************************/
/**
Use a special protocol to update the firmware in Copley CANopen devices.
Note that this is not part of normal operation, but rather a special
utility function which allows the device firmware to be updated if
necessary.

Only firmware files produced by Copley Controls Corp. should be used
to update Copley devices.  Attempting to update a device with incorrectly 
formatted firmware files will render the it inoperable.

If an error occurs during the download of new firmware, it may be necessary
to reprogram the device through it's serial port.  The CME-2 software 
can be used to recover an device in this case.

@param fw The firmware object holding the data to be programmed.
*/
/***************************************************************************/
const Error *CopleyNode::FirmwareUpdate( Firmware &fw )
{
   const Error *err;
   int32 magic;

   // Upload the 'magic number' used to program firmware
   err = sdo.Upld32( OBJID_FIRMWARE, 0, magic );

   // Now, write that same value back to the amp.  This will
   // put us in a special mode
   if( !err ) err = sdo.Dnld32( OBJID_FIRMWARE, 0, magic );
   if( err ) return err;

   FirmwareUpdater up( sdo );

   // Disable the SDO object.  It's no longer useful
   err = sdo.DisableReceiver();
   if( !err ) err = up.EnableReceiver();

   // Send a command telling the amp to erase it's program flash
   if( !err ) err = up.SendCmd( 0x0101, 5000 );
   if( err ) return err;

   // OK, the program flash in the amplifier is gone.  At this point
   // if power fails, or the computer crashes, the amplifier will be
   // unable to run.
   //
   // It should however be possible to reprogram the amplifier using
   // the CME-2 program supplied by Copley Controls.  This program 
   // uses the RS-232 interface on the amplifier to reprogram the amp.
   uint32 start  = fw.getStart();
   uint32 length = fw.getLength();
   uint16 *data  = fw.getData();

   int errCt = 0;

   for( uint32 i=0; i<length; )
   {
      uint16 x[3];
      x[0] = (uint)(start+i);
      x[1] = data[i++];

      if( i < length )
      {
	 x[2] = data[i++];
	 err = up.SendCmd( 0x103, 3, x, 500 );
      }
      else
	 err = up.SendCmd( 0x103, 2, x, 500 );

      if( !err ) 
	 errCt = 0;
      else if( ++errCt > 10 )
	 return err;
      else
	 i = up.data - start;

      fw.progress( i );
   }

   if( !err ) err = up.SendCmd( 0x0104, 1000 );
   if( err ) return err;

   up.SendCmd( 0x0100, 0 );
   return 0;
}

