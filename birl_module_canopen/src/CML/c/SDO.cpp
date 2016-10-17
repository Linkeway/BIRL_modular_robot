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

This file contains the code used to implement the CANopen SDO objects.
*/

#include "CML.h"

CML_NAMESPACE_USE();

/**************************************************
* SDO Error objects
**************************************************/
CML_NEW_ERROR( SDO_Error, NoAbortCode,       "SDO Aborted" );
CML_NEW_ERROR( SDO_Error, Togglebit,         "SDO Abort: Toggle bit not alternated." );
CML_NEW_ERROR( SDO_Error, Timeout,           "SDO Abort: SDO Protocol timed out." );
CML_NEW_ERROR( SDO_Error, Bad_scs,           "SDO Abort: Client/Server command specifier not known." );
CML_NEW_ERROR( SDO_Error, Block_size,        "SDO Abort: Invalid block size." );
CML_NEW_ERROR( SDO_Error, Block_seq,         "SDO Abort: Invalid block sequence." );
CML_NEW_ERROR( SDO_Error, Block_crc,         "SDO Abort: CRC error." );
CML_NEW_ERROR( SDO_Error, Memory,            "SDO Abort: Memory allocation error." );
CML_NEW_ERROR( SDO_Error, Access,            "SDO Abort: Unsupported object access." );
CML_NEW_ERROR( SDO_Error, Writeonly,         "SDO Abort: Object is write only." );
CML_NEW_ERROR( SDO_Error, Readonly,          "SDO Abort: Object is read only." );
CML_NEW_ERROR( SDO_Error, Bad_object,        "SDO Abort: Object does not exist." );
CML_NEW_ERROR( SDO_Error, Pdo_map,           "SDO Abort: Object can not be mapped to PDO." );
CML_NEW_ERROR( SDO_Error, Pdo_length,        "SDO Abort: PDO length would be exceeded." );
CML_NEW_ERROR( SDO_Error, Bad_param,         "SDO Abort: General parameter error." );
CML_NEW_ERROR( SDO_Error, Incompatible,      "SDO Abort: General internal incompatibility." );
CML_NEW_ERROR( SDO_Error, Hardware,          "SDO Abort: Hardware failure." );
CML_NEW_ERROR( SDO_Error, Bad_length,        "SDO Abort: Data length incorrect." );
CML_NEW_ERROR( SDO_Error, Too_long,          "SDO Abort: Data length too long." );
CML_NEW_ERROR( SDO_Error, Too_short,         "SDO Abort: Data length too short." );
CML_NEW_ERROR( SDO_Error, Subindex,          "SDO Abort: Sub-index does not exist." );
CML_NEW_ERROR( SDO_Error, Param_range,       "SDO Abort: Parameter range error." );
CML_NEW_ERROR( SDO_Error, Param_high,        "SDO Abort: Parameter value too high." );
CML_NEW_ERROR( SDO_Error, Param_low,         "SDO Abort: Parameter value too low." );
CML_NEW_ERROR( SDO_Error, Min_max,           "SDO Abort: Maximum value less then minimum." );
CML_NEW_ERROR( SDO_Error, General,           "SDO Abort: General error." );
CML_NEW_ERROR( SDO_Error, Transfer,          "SDO Abort: Data transfer error." );
CML_NEW_ERROR( SDO_Error, Transfer_Local,    "SDO Abort: Data transfer error; local control." );
CML_NEW_ERROR( SDO_Error, Transfer_State,    "SDO Abort: Data transfer error; device state." );
CML_NEW_ERROR( SDO_Error, OD_Gen_Fail,       "SDO Abort: Object dictionary generation failure." );
CML_NEW_ERROR( SDO_Error, Unknown,           "SDO Abort: Unknown abort code" );


/**************************************************
* States used internally by the SDO object
**************************************************/
#define SDO_STATE_IDLE                   0     // SDO is idle
#define SDO_STATE_DONE                   1     // SDO transfer finished
#define SDO_STATE_SENT_DNLD_INIT         2     // SDO download init was sent.
#define SDO_STATE_SENT_DOWNLOAD          3     // SDO download sent
#define SDO_STATE_SENT_UPLD_INIT         4     // SDO upload init was sent.
#define SDO_STATE_SENT_UPLD_RQST         5     // SDO upload request sent
#define SDO_STATE_SENT_BUP_INIT          6     // Block upload init sent
#define SDO_STATE_SENT_BUP_START         7     // Block upload data expected
#define SDO_STATE_SENT_BUP_LAST          8     // Block upload done

#define SDO_STATE_SEND_ABORT            10     // I need to send an abort
#define SDO_STATE_SEND_DATA             11     // I need to send more data
#define SDO_STATE_SEND_UPLD             12     // I need to send an upload request
#define SDO_STATE_SEND_BUP_START        13     // I need to send a block upload start
#define SDO_STATE_SEND_BUP_NEXT         14     // request next block upload block
#define SDO_STATE_SEND_BUP_LAST         15     // Last block has been uploaded
#define SDO_STATE_SEND_BUP_DONE         16     // Upload finished, send confirmation

/***************************************************************************/
/**
Initialize the CANopen Service Data Object (SDO).
@param canOpen The CANopen network object that this SDO is associated with
@param xmit The COB ID used when transmitting SDO frames
@param recv The COB ID used for receive frames
@param to The timeout (milliseconds) for use with this SDO.
*/
/***************************************************************************/
SDO::SDO( CanOpen &canOpen, uint32 xmit, uint32 recv, int32 to )
{
   Init( canOpen, xmit, recv, to );
}

/***************************************************************************/
/**
Initialize a CANopen Service Data Object (SDO).
@param canOpen The CANopen network object that this SDO is associated with
@param xmit The COB ID used when transmitting SDO frames
@param recv The COB ID used for receive frames
@param to The timeout (milliseconds) for use with this SDO.
@return A valid CANopen error object
*/
/***************************************************************************/
const Error *SDO::Init( CanOpen &canOpen, uint32 xmit, uint32 recv, int32 to)
{
   xmitID = xmit;
   state = SDO_STATE_IDLE;
   timeout = to;
   lastErr = 0;

   blkUpldOK = false;
   blkDnldOK = false;

   const Error *err = Receiver::Init( canOpen, recv );
   if( err ) return err;
   EnableReceiver();

   return 0;
}

/***************************************************************************/
/**
Download a 32-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The data to be downloaded
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Dnld32( int16 index, int16 sub, uint32 data )
{
   byte buff[4];
   buff[0] = ByteCast(data);
   buff[1] = ByteCast(data>>8);
   buff[2] = ByteCast(data>>16);
   buff[3] = ByteCast(data>>24);
   return Download( index, sub, 4, buff );
}

/***************************************************************************/
/**
Upload a 32-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The uploaded data will be returned here.
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Upld32( int16 index, int16 sub, uint32 &data )
{
   int32 size = 4;
   byte buff[4];
   buff[0] = buff[1] = buff[2] = buff[3] = 0;

   const Error *err = Upload( index, sub, size, buff );

   data = bytes_to_uint32( buff );

   return err;
}

/***************************************************************************/
/**
Download a 16-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The data to be downloaded
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Dnld16( int16 index, int16 sub, uint16 data )
{
   byte buff[2];
   buff[0] = ByteCast(data);
   buff[1] = ByteCast(data>>8);
   return Download( index, sub, 2, buff );
}

/***************************************************************************/
/**
Upload a 16-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The uploaded data will be returned here.
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Upld16( int16 index, int16 sub, uint16 &data )
{
   int32 size = 2;
   byte buff[2];
   buff[0] = buff[1] = 0;

   const Error *err = Upload( index, sub, size, buff );

   data = bytes_to_uint16(buff);
   return err;
}

/***************************************************************************/
/**
Download a 8-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The data to be downloaded
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Dnld8( int16 index, int16 sub, uint8 data )
{
   return Download( index, sub, 1, &data );
}

/***************************************************************************/
/**
Upload a 8-bit value using this SDO.
@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data The uploaded data will be returned here.
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::Upld8( int16 index, int16 sub, uint8 &data )
{
   int32 size = 1;
   return Upload( index, sub, size, &data );
}

/***************************************************************************/
/**
Download a visible string type using the SDO.  The string is assumed
to be null terminated.

@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param data A null terminated string to be downloaded.
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::DnldString( int16 index, int16 sub, char *data )
{
   // Find the string length
   int32 i;
   for( i=0; data[i]; i++ );

   return Download( index, sub, i, data );
}

/***************************************************************************/
/**
Upload a visible string type from the SDO.  The only difference between
this function and the lower level Upload function is that this function
guarantees that there will be a zero character at the end of the string.

@param index The index of the object in the object dictionary
@param sub The sub-index of the object in the object dictionary
@param len Holds the size of the buffer on entry, and the
           length of the downloaded data on return.
@param data The uploaded string will be returned here.
@return A valid CANopen error code.
*/
/***************************************************************************/
const Error *SDO::UpldString( int16 index, int16 sub, int32 &len, char *data )
{
   len--;

   const Error *err = Upload( index, sub, len, data );
   if( err ) return err;

   data[len] = 0;
   return 0;
}

/***************************************************************************/
/**
Download data using this SDO.  The passed array of data is downloaded to the
object dictionary of a node on the CANopen network using this SDO.
@param index The index of the object to be downloaded.
@param sub The sub-index of the object to be downloaded.
@param size The number of bytes of data to be downloaded
@param data A character array holding the data to be downloaded.
@return A valid CANopen error object.
*/
/***************************************************************************/
const Error *SDO::Download( int16 index, int16 sub, int32 size, byte *data )
{
   CanFrame frame;
   const Error *err;

   // Use a block download if it makes sense to do so
   if( blkDnldOK && size >= SDO_BLK_DNLD_THRESHOLD )
      return BlockDnld( index, sub, size, data );

   // Make sure the SDO has been initialized
   if( !co ) return &CanOpenError::NotInitialized;

   mutex.Lock();

   if( state != SDO_STATE_IDLE )
      err = &CanOpenError::SDO_Busy;

   else if( size <= 0 )
      err = &CanOpenError::BadParam;

   else
   {
      // send an "Initiate SDO download" message.
      frame.id = xmitID;
      frame.type = CAN_FRAME_DATA;
      frame.length = 8;

      // Copy the object multiplexor to the frame
      // and also make a local copy.
      frame.data[1] = mplex[0] = ByteCast(index);
      frame.data[2] = mplex[1] = ByteCast(index>>8);
      frame.data[3] = mplex[2] = ByteCast(sub);

      // If the data size is <= 4 bytes, then send an expedited download
      if( size <= 4 )
      {
	 frame.data[0] = 0x23 | ((4-size)<<2);

	 int32 i;
	 for( i=0; i<size; i++ ) frame.data[i+4] = ByteCast(data[i]);
	 for( ; i<4; i++ )       frame.data[i+4] = 0;

	 remain = 0;
      }

      // Otherwise, send a normal init
      else
      {
	 frame.data[0] = 0x21;
	 frame.data[4] = ByteCast(size);
	 frame.data[5] = ByteCast(size>>8);
	 frame.data[6] = ByteCast(size>>16);
	 frame.data[7] = ByteCast(size>>24);

	 remain = size;
      }

      // Keep a pointer to the data buffer and reset the toggle bit
      dataPtr = data;
      toggle = 0;

      state = SDO_STATE_SENT_DNLD_INIT;
      err = co->Xmit( frame, timeout );
      if( err ) state = SDO_STATE_IDLE;
   }

   // If the download was started successfully, sit waiting for the
   // state to change to one that indicates success or failure.
   if( !err ) err = WaitForTransfer( size );

   mutex.Unlock();
   return err;
}

/***************************************************************************/
/**
Upload data using this SDO.  The value of the object is uploaded from the
object dictionary of a node on the CANopen network using this SDO.  The 
results of the upload are stored in the passed buffer.

@param index The index of the object to be uploaded.
@param sub The sub-index of the object to be uploaded.
@param size On entry, this gives the maximum number of bytes of data to 
       be uploaded.  On successful return, it gives the actual number of
		 bytes received.
@param data A character array which will store the uploaded data.
@return A valid CANopen error object.
*/
/***************************************************************************/
const Error *SDO::Upload( int16 index, int16 sub, int32 &size, byte *data )
{
   CanFrame frame;
   const Error *err;

   // Use a block upload if it makes sense to do so
   if( blkUpldOK && size >= SDO_BLK_UPLD_THRESHOLD )
      return BlockUpld( index, sub, size, data );

   // Make sure the SDO has been initialized
   if( !co ) return &CanOpenError::NotInitialized;

   mutex.Lock();

   if( state != SDO_STATE_IDLE )
      err = &CanOpenError::SDO_Busy;

   else if( size <= 0 )
      err = &CanOpenError::BadParam;

   else
   {
      // send an "Initiate SDO upload" message.
      frame.id = xmitID;
      frame.type = CAN_FRAME_DATA;
      frame.length = 8;

      frame.data[0] = 0x40;

      // Copy the object multiplexor to the frame
      // and also make a local copy.
      frame.data[1] = mplex[0] = ByteCast(index);
      frame.data[2] = mplex[1] = ByteCast(index>>8);
      frame.data[3] = mplex[2] = ByteCast(sub);

      // Clear out the reserved bytes
      for( int i=4; i<8; i++ )
	 frame.data[i] = 0;

      remain = size;

      // Keep a pointer to the data buffer and reset the toggle bit
      dataPtr = data;
      toggle = 0;

      state = SDO_STATE_SENT_UPLD_INIT;
      err = co->Xmit( frame, timeout );
      if( err ) state = SDO_STATE_IDLE;
   }

   // If the upload was started successfully, sit waiting for the
   // state to change to one that indicates success or failure.
   if( !err ) err = WaitForTransfer( size );
   mutex.Unlock();
   return err;
}

/***************************************************************************/
/**
Upload data using this SDO.  This function uses a block upload protocol
which makes sending large blocks of data more efficient.  The specified
object is upload from the CANopen node's object dictionary and stored in    
the array passed to this function.
 
@param index The index of the object to be uploaded.
@param sub The sub-index of the object to be uploaded.
@param size On entry, this should be the maximum number of bytes
       to upload, on successful return, this is the number of bytes
		 actually received.
@param data A character array which will store the uploaded data.
@return A valid CANopen error object.
*/
/***************************************************************************/
const Error *SDO::BlockUpld( int16 index, int16 sub, int32 &size, byte *data )
{
   CanFrame frame;
   const Error *err;

   // Make sure the SDO has been initialized
   if( !co ) return &CanOpenError::NotInitialized;

   mutex.Lock();

   if( state != SDO_STATE_IDLE )
      err = &CanOpenError::SDO_Busy;

   else if( size <= 0 )
      err = &CanOpenError::BadParam;

   else
   {
      // send an "Initiate block upload" message.
      frame.id = xmitID;
      frame.type = CAN_FRAME_DATA;
      frame.length = 8;

      // BUG: I don't support CRC checking at the moment
      frame.data[0] = 0xA0;

      // Copy the object multiplexor to the frame
      // and also make a local copy.
      frame.data[1] = mplex[0] = ByteCast(index);
      frame.data[2] = mplex[1] = ByteCast(index>>8);
      frame.data[3] = mplex[2] = ByteCast(sub);

      // For now, I use 127 segments/block and allow
      // drop back to normal update if the number of
      // actual bytes is less then my threshold.
      blockSize = 127;
      frame.data[4] = blockSize;
      frame.data[5] = SDO_BLK_UPLD_THRESHOLD-1;

      // Clear out the reserved bytes
      frame.data[6] = 0;
      frame.data[7] = 0;

      remain = size;
      toggle = 0;

      // Keep a pointer to the data buffer
      dataPtr = data;

      state = SDO_STATE_SENT_BUP_INIT;
      err = co->Xmit( frame, timeout );
      if( err ) state = SDO_STATE_IDLE;
   }

   if( !err ) err = WaitForTransfer( size );
   mutex.Unlock();
   return err;
}

/***************************************************************************/
/**
Download data using this SDO.  This function uses a block download protocol
which makes sending large blocks of data more efficient.  The data passed
to this function is downloaded to the object dictionary of the CANopen node

@param index The index of the object to be downloaded.
@param sub The sub-index of the object to be downloaded.
@param size The number of bytes of data to be downloaded
@param data A character array holding the data to be downloaded.
@return A valid CANopen error object.
*/
/***************************************************************************/
const Error *SDO::BlockDnld( int16 index, int16 sub, int32 size, byte *data )
{
   // This isn't supported yet, just return an error
   return &CanOpenError::NotSupported;
}

/***************************************************************************/
/**
Wait for the current SDO transfer to finish.  This is an internal function 
used by all the SDO transfer modes (upload, download, block upload, block
download).  It simply waits on the SDO's semaphore until either the SDO
transfer times out, or finishes either successfully or not.

@param size Used to pass back the size of the transfer.
@return An error object.
*/
/***************************************************************************/
const Error *SDO::WaitForTransfer( int32 &size )
{
   const Error *err;

   while( 1 )
   {
      err = sem.Get( timeout );

      if( err || state == SDO_STATE_DONE )
	 break;
   }

   if( err )
   {
      if( err == &ThreadError::Timeout )
	 err = &CanOpenError::SDO_Timeout;
      else
	 err = &CanOpenError::SDO_Unknown;

      SendAbort( SDO_ABORT_TIMEOUT );
   }

   else
   {
      err = lastErr;
      size = count;
   }

   state = SDO_STATE_IDLE;

   return err;
}

/***************************************************************************/
/**
Process a newly received CAN frame addressed to this SDO object.  This
is called from the CanOpen receive thread when a frame addressed to this
SDO is received over the network.  This frame is processed based on the
state of the SDO.
@param frame The frame to be processed
@return This function always returns 1.
*/
/***************************************************************************/
int SDO::NewFrame( CanFrame &frame )
{
   // Ignore messages when idle
   if( state == SDO_STATE_IDLE )
      return 1;

   // Ignore remote frames
   if( frame.type != CAN_FRAME_DATA )
      return 1;

   // Pull the server command specifier out of the message.
   // If no data was passed, then just set an illegal scs value.
   int16 scs = (frame.length<1) ? -1 : (int16)(7 & (frame.data[0]>>5));

   // This is filled in if we need to send an abort
   int32 abortCode = 0;

   /**************************************************
    * Handle block upload/download fall backs.  These
    * are responses to block up/downloads that treat
    * them as normal up/downloads.
    **************************************************/
   if( (state == SDO_STATE_SENT_BUP_INIT) && (scs==2) )
      state = SDO_STATE_SENT_UPLD_INIT;

   /**************************************************
    * Handle aborts.  An abort has an scs of 4, but 
    * for a block upload that could be valid, so check
    * for a byte value of 0x80 in that case.
    **************************************************/
   int isAbort = (frame.data[0]==0x80) || ( (scs==4) && (state!=SDO_STATE_SENT_BUP_START) );

   if( isAbort )
   {
      lastErr = getAbortRcvdErr( frame );

      // Either way, I'm done after receiving an abort.
      state = SDO_STATE_DONE;
      sem.Put();
      return 1;
   }

   /**************************************************
    * First, determine what to do based on my state
    * and the data contained in the message.
    **************************************************/
   switch( state )
   {
      /**************************************************
       * Handle the response to a upload init message
       **************************************************/
      case SDO_STATE_SENT_UPLD_INIT:
      {
	 // I expect an scs value of 2 (init upload response).
	 if( scs != 2 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }

	 int32 upldSize;

	 // Check the multiplexor sent with the message
	 int muxOK = (frame.data[1] == mplex[0] && 
	       frame.data[2] == mplex[1] && 
	       frame.data[3] == mplex[2]);

	 // If the multiplexor was not right, abort the transfer
	 if( !muxOK )
	 {
	    lastErr = &CanOpenError::SDO_BadMuxRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_GENERAL_ERR;
	    break;
	 }

	 // Grab the size information passed with 
	 // the message (if any is passed).
	 if( frame.data[0] & 1 )
	 {
	    // Expedited transfer size info
	    if( frame.data[0] & 2 )
	       upldSize = 4 - (3&(frame.data[0]>>2));

	    // Normal transfer size info
	    else
	    {
	       upldSize = bytes_to_int32( &frame.data[4] );
	    }
	 }

	 // If no size info was specified, just 
	 // set my size to -1.
	 else
	    upldSize = -1;

	 // If this is an expedited transfer, copy the data
	 // (as much as I can handle) to my buffer.
	 if( frame.data[0] & 2 )
	 {
	    int ct = (upldSize<0) ? 4 : upldSize;

	    // Clip the size to the available memory
	    if( ct > remain ) ct = remain;

	    for( int i=0; i<ct; i++ )
	       *dataPtr++ = frame.data[i+4];

	    remain -= ct;
	    count = ct;

	    lastErr = 0;
	    state = SDO_STATE_DONE;
	 }

	 // For normal transfers I'll send an upload
	 // request.  Note that I don't abort the 
	 // transfer if the upload size doesn't match
	 // my buffer size.  I'll just grab what ever
	 // data I can handle.
	 else
	 {
	    // If the upload size was specified, and is 
	    // less then my buffer size, I'll set my
	    // remaining count equal to it.
	    if( upldSize > 0 && upldSize < remain )
	       remain = upldSize;
	    state = SDO_STATE_SEND_UPLD;
	    count = 0;
	 }

	 break;
      }

      /**************************************************
       * Handle the response to an upload segment request
       **************************************************/
      case SDO_STATE_SENT_UPLD_RQST:
      {
	 // I expect an SCS of 0 indicating uploaded data.
	 if( scs != 0 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }

	 // Check the toggle bit
	 int expected = toggle ? 0 : 0x10;
	 if( (frame.data[0] & 0x10) != expected )
	 {
	    abortCode = SDO_ABORT_TOGGLEBIT;
	    lastErr = &SDO_Error::Togglebit;
	    state = SDO_STATE_SEND_ABORT;
	    break;
	 }

	 // If the number of bytes of data passed in this
	 // message was specified, then decode it.
	 // Otherwise, assume 7.
	 int ct = 7 - (7&(frame.data[0]>>1));

	 // If the number of bytes sent in the message is
	 // greater then my buffer size, clip it.
	 if( ct > remain ) ct = remain;

	 // Copy the data
	 for( int i=1; i<=ct; i++ )
	    *dataPtr++ = frame.data[i];

	 remain -= ct;
	 count += ct;

	 // If this was the last message then I'm done
	 if( frame.data[0] & 1 )
	 {
	    lastErr = 0;
	    state = SDO_STATE_DONE;
	 }

	 // Otherwise, if my buffer is full I'll
	 // abort the transfer.  We don't need your
	 // stinkin data!
	 else if( !remain )
	 {
	    lastErr = 0;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_TOO_LONG;
	 }

	 // I need more data, ask for some.
	 else
	    state = SDO_STATE_SEND_UPLD;

	 break;
      }

      /**************************************************
       * Handle the response to an download init message
       **************************************************/
      case SDO_STATE_SENT_DNLD_INIT:
      {
	 // Check the multiplexor sent with the message
	 int muxOK = (frame.data[1] == mplex[0] && 
	       frame.data[2] == mplex[1] && 
	       frame.data[3] == mplex[2]);

	 // I expect an scs value of 3 (init download response).
	 if( scs != 3 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }


	 // If the multiplexor was not right, abort the transfer
	 if( !muxOK )
	 {
	    lastErr = &CanOpenError::SDO_BadMuxRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_GENERAL_ERR;
	 }

	 // If I have more data to send, then pass the next block
	 else if( remain > 0 )
	    state = SDO_STATE_SEND_DATA;

	 // Otherwise, I'm done
	 else
	 {
	    lastErr = 0;
	    state = SDO_STATE_DONE;
	 }

	 break;
      }

      /**************************************************
       * Handle the response to a download segment
       **************************************************/
      case SDO_STATE_SENT_DOWNLOAD:
      {
	 // I expect an scs value of 1 (download response).
	 if( scs != 1 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }


	 // Check the toggle bit
	 int expected = toggle ? 0 : 0x10;
	 if( (frame.data[0] & 0x10) != expected )
	 {
	    abortCode = SDO_ABORT_TOGGLEBIT;
	    lastErr = &SDO_Error::Togglebit;
	    state = SDO_STATE_SEND_ABORT;
	 }

	 // If I have more data to send, then pass the next block
	 else if( remain > 0 )
	    state = SDO_STATE_SEND_DATA;

	 // Otherwise, I'm done
	 else
	 {
	    lastErr = 0;
	    state = SDO_STATE_DONE;
	 }

	 break;
      }

      /**************************************************
       * Handle the response to a block upload init.
       * Note that I don't have to worry about fall back 
       * to normal upload mode, I take care of that above.
       **************************************************/
      case SDO_STATE_SENT_BUP_INIT:
      {
	 int32 upldSize;

	 // Check the multiplexor sent with the message
	 int muxOK = (frame.data[1] == mplex[0] && 
	       frame.data[2] == mplex[1] && 
	       frame.data[3] == mplex[2]);

	 // I expect an scs value of 6 (block upload response).
	 if( scs != 6 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }

	 // Make sure the sub-code was valid
	 if( frame.data[0] & 1 )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }

	 // If the multiplexor was not right, abort the transfer
	 if( !muxOK )
	 {
	    lastErr = &CanOpenError::SDO_BadMuxRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_GENERAL_ERR;
	    break;
	 }

	 // Grab the size information passed with 
	 // the message (if any is passed).
	 if( frame.data[0] & 2 )
	 {
	    upldSize = bytes_to_int32( &frame.data[4] );
	 }

	 // If no size info was specified, just 
	 // set my size to -1.
	 else
	    upldSize = -1;

	 // If the upload size was specified, and is 
	 // less then my buffer size, I'll set my
	 // remaining count equal to it.
	 if( upldSize > 0 && upldSize < remain )
	    remain = upldSize;
	 state = SDO_STATE_SEND_BUP_START;
	 count = 0;

	 break;
      }

      /**************************************************
       * Handle a block upload.
       **************************************************/
      case SDO_STATE_SENT_BUP_START:
      {
	 // Find the sequence number for this segment
	 int seq = frame.data[0] & 0x7F;

	 // If this is my expected sequence number, 
	 // copy the data.
	 if( seq == lastBlkSeq+1 )
	 {
	    int ct = (remain<7) ? remain : 7;

	    for( int i=1; i<=ct; i++ )
	       *dataPtr++ = frame.data[i];

	    remain -= ct;
	    count += ct;

	    lastBlkSeq++;
	 }

	 // If this was the last sequence, then send a response
	 if( frame.data[0] & 0x80 )
	 {
	    // If all data was received correctly, end the transfer
	    if( lastBlkSeq == seq )
	       state = SDO_STATE_SEND_BUP_LAST;

	    // Otherwise, re-request
	    else
	       state = SDO_STATE_SEND_BUP_NEXT;
	 }

	 // Send a response if this was the last
	 // segment in the block
	 else if( seq == blockSize )
	    state = SDO_STATE_SEND_BUP_NEXT;

	 break;
      }

      /**************************************************
       * All blocks have been received, waiting for end
       * message.
       **************************************************/
      case SDO_STATE_SENT_BUP_LAST:
      {
	 if( (scs != 6) || ((frame.data[0]&3) != 1) )
	 {
	    lastErr = &CanOpenError::SDO_BadMsgRcvd;
	    state = SDO_STATE_SEND_ABORT;
	    abortCode = SDO_ABORT_BAD_SCS;
	    break;
	 }

	 lastErr = 0;
	 state = SDO_STATE_SEND_BUP_DONE;
	 break;
      }
   }

   /**************************************************
    * Now that I've figured out what to do (and updated
    * the state variable to indicate what it should be)
    * just do it!
    **************************************************/
   frame.id = xmitID;
   frame.length = 8;

   switch( state )
   {
      case SDO_STATE_SEND_ABORT:
	 {
	    // Send an abort frame 
	    SendAbort( abortCode );

	    // Now, update my state to done.
	    state = SDO_STATE_DONE;

	    // Fall through to done processing
	 }

      case SDO_STATE_DONE:
	 // Indicate that I've finished.
	 sem.Put();
	 break;

      case SDO_STATE_SEND_DATA:
	 {
	    // Send the next data block
	    int ct = (remain > 7 ) ? 7 : remain;
	    remain -= ct;

	    frame.data[0] = (7-ct)<<1;
	    if( toggle )  frame.data[0] |= 0x10;
	    if( !remain ) frame.data[0] |= 0x01;

	    for( int i=1; i<=ct; i++ )
	       frame.data[i] = ByteCast(*dataPtr++);

	    toggle = !toggle;
	    state = SDO_STATE_SENT_DOWNLOAD;
	    co->Xmit( frame, timeout );
	    break;
	 }

      case SDO_STATE_SEND_UPLD:
	 {
	    frame.data[0] = toggle ? 0x70 : 0x60;

	    // Clear out reserved bytes
	    for( int i=1; i<8; i++ )
	       frame.data[i] = 0;

	    toggle = !toggle;
	    state = SDO_STATE_SENT_UPLD_RQST;
	    co->Xmit( frame, timeout );
	    break;
	 }

      case SDO_STATE_SEND_BUP_START:
	 {
	    frame.data[0] = 0xA3;
	    for( int i=1; i<8; i++ )
	       frame.data[i] = 0;

	    lastBlkSeq = 0;
	    state = SDO_STATE_SENT_BUP_START;
	    co->Xmit( frame, timeout );
	    break;
	 }

      case SDO_STATE_SEND_BUP_NEXT:
      case SDO_STATE_SEND_BUP_LAST:
	 {
	    frame.data[0] = 0xA2;
	    frame.data[1] = ByteCast(lastBlkSeq);
	    blockSize = 127;
	    frame.data[2] = blockSize;

	    for( int i=3; i<8; i++ )
	       frame.data[i] = 0;

	    if( state == SDO_STATE_SEND_BUP_LAST )
	       state = SDO_STATE_SENT_BUP_LAST;
	    else
	       state = SDO_STATE_SENT_BUP_START;

	    lastBlkSeq = 0;
	    co->Xmit( frame, timeout );
	    break;
	 }

      case SDO_STATE_SEND_BUP_DONE:
	 {
	    frame.data[0] = 0xA1;
	    for( int i=1; i<8; i++ )
	       frame.data[i] = 0;

	    co->Xmit( frame, timeout );
	    state = SDO_STATE_DONE;
	    sem.Put();
	    break;
	 }
   }

   return 1;
}

/***************************************************************************/
/**
Send an abort frame.
*/
/***************************************************************************/
void SDO::SendAbort( int32 abortCode )
{
   // Make sure the SDO has been initialized
   if( !co ) return;

   CanFrame frame;

   frame.id = xmitID;
   frame.type = CAN_FRAME_DATA;
   frame.length = 8;
   frame.data[0] = 0x80;
   frame.data[1] = mplex[0];
   frame.data[2] = mplex[1];
   frame.data[3] = mplex[2];
   frame.data[4] = ByteCast(abortCode);
   frame.data[5] = ByteCast(abortCode>>8);
   frame.data[6] = ByteCast(abortCode>>16);
   frame.data[7] = ByteCast(abortCode>>24);

   co->Xmit( frame, timeout );
}

/***************************************************************************/
/**
Translate the passed SDO abort code into an error message.
*/
/***************************************************************************/
const Error *SDO::getAbortRcvdErr( CanFrame &frame )
{
   int32 code = bytes_to_int32( &frame.data[4] );

   switch( code )
   {
      case 0:                                return &SDO_Error::NoAbortCode;
      case SDO_ABORT_TOGGLEBIT:              return &SDO_Error::Togglebit;
      case SDO_ABORT_TIMEOUT:                return &SDO_Error::Timeout;
      case SDO_ABORT_BAD_SCS:                return &SDO_Error::Bad_scs;
      case SDO_ABORT_BLOCK_SIZE:             return &SDO_Error::Block_size;
      case SDO_ABORT_BLOCK_SEQ:              return &SDO_Error::Block_seq;
      case SDO_ABORT_BLOCK_CRC:              return &SDO_Error::Block_crc;
      case SDO_ABORT_MEMORY:                 return &SDO_Error::Memory;
      case SDO_ABORT_ACCESS:                 return &SDO_Error::Access;
      case SDO_ABORT_WRITEONLY:              return &SDO_Error::Writeonly;
      case SDO_ABORT_READONLY:               return &SDO_Error::Readonly;
      case SDO_ABORT_BAD_OBJECT:             return &SDO_Error::Bad_object;
      case SDO_ABORT_PDO_MAP:                return &SDO_Error::Pdo_map;
      case SDO_ABORT_PDO_LENGTH:             return &SDO_Error::Pdo_length;
      case SDO_ABORT_BAD_PARAM:              return &SDO_Error::Bad_param;
      case SDO_ABORT_INCOMPATIBLE:           return &SDO_Error::Incompatible;
      case SDO_ABORT_HARDWARE:               return &SDO_Error::Hardware;
      case SDO_ABORT_BAD_LENGTH:             return &SDO_Error::Bad_length;
      case SDO_ABORT_TOO_LONG:               return &SDO_Error::Too_long;
      case SDO_ABORT_TOO_SHORT:              return &SDO_Error::Too_short;
      case SDO_ABORT_SUBINDEX:               return &SDO_Error::Subindex;
      case SDO_ABORT_PARAM_RANGE:            return &SDO_Error::Param_range;
      case SDO_ABORT_PARAM_HIGH:             return &SDO_Error::Param_high;
      case SDO_ABORT_PARAM_LOW:              return &SDO_Error::Param_low;
      case SDO_ABORT_MIN_MAX:                return &SDO_Error::Min_max;
      case SDO_ABORT_GENERAL_ERR:            return &SDO_Error::General;
      case SDO_ABORT_TRANSFER:               return &SDO_Error::Transfer;      
      case SDO_ABORT_TRANSFER_LOCAL:         return &SDO_Error::Transfer_Local;
      case SDO_ABORT_TRANSFER_STATE:         return &SDO_Error::Transfer_State;
      case SDO_ABORT_OD_GEN_FAIL:            return &SDO_Error::OD_Gen_Fail;
      default:                               return &SDO_Error::Unknown;
   }
}


