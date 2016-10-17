/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This header file defines the classes used to communicate
to CANopen nodes using Service Data Objects (SDOs).

*/

#ifndef _DEF_INC_CO_SDO
#define _DEF_INC_CO_SDO

#include "CML_Settings.h"
#include "CML_CanOpen.h"

CML_NAMESPACE_START()

/**************************************************
* SDO abort codes
**************************************************/
#define SDO_ABORT_TOGGLEBIT              0x05030000
#define SDO_ABORT_TIMEOUT                0x05040000
#define SDO_ABORT_BAD_SCS                0x05040001
#define SDO_ABORT_BLOCK_SIZE             0x05040002 
#define SDO_ABORT_BLOCK_SEQ              0x05040003 
#define SDO_ABORT_BLOCK_CRC              0x05040004 
#define SDO_ABORT_MEMORY                 0x05040005 
#define SDO_ABORT_ACCESS                 0x06010000
#define SDO_ABORT_WRITEONLY              0x06010001
#define SDO_ABORT_READONLY               0x06010002
#define SDO_ABORT_BAD_OBJECT             0x06020000
#define SDO_ABORT_PDO_MAP                0x06040041
#define SDO_ABORT_PDO_LENGTH             0x06040042
#define SDO_ABORT_BAD_PARAM              0x06040043
#define SDO_ABORT_INCOMPATIBLE           0x06040047
#define SDO_ABORT_HARDWARE               0x06060000
#define SDO_ABORT_BAD_LENGTH             0x06070010
#define SDO_ABORT_TOO_LONG               0x06070012
#define SDO_ABORT_TOO_SHORT              0x06070013
#define SDO_ABORT_SUBINDEX               0x06090011
#define SDO_ABORT_PARAM_RANGE            0x06090030
#define SDO_ABORT_PARAM_HIGH             0x06090031
#define SDO_ABORT_PARAM_LOW              0x06090032
#define SDO_ABORT_MIN_MAX                0x06090036
#define SDO_ABORT_GENERAL_ERR            0x08000000
#define SDO_ABORT_TRANSFER               0x08000020
#define SDO_ABORT_TRANSFER_LOCAL         0x08000021
#define SDO_ABORT_TRANSFER_STATE         0x08000022
#define SDO_ABORT_OD_GEN_FAIL            0x08000023

/// SDO downloads of this size (byte) or greater are more
/// efficiently handled using a block download.
#define SDO_BLK_DNLD_THRESHOLD    15

/// SDO uploads of this size (byte) or greater are more
/// efficiently handled using a block upload.
#define SDO_BLK_UPLD_THRESHOLD    15


/**
This class represents SDO errors.  There is one static
member for each SDO abort code.
*/
class SDO_Error: public CanOpenError
{
public:
   /// No abort code was specified with the SDO Abort message
   static const SDO_Error NoAbortCode;
   /// SDO Abort - toggle bit error
   static const SDO_Error Togglebit;
   /// SDO Abort - Timeout
   static const SDO_Error Timeout;
   /// SDO Abort - Bad SCS code
   static const SDO_Error Bad_scs;
   /// SDO Abort - Bad Block size
   static const SDO_Error Block_size;
   /// SDO Abort - Block sequence error
   static const SDO_Error Block_seq;
   /// SDO Abort - Block CRC error
   static const SDO_Error Block_crc;
   /// SDO Abort - Memory allocation failure
   static const SDO_Error Memory;
   /// SDO Abort - Access mode error
   static const SDO_Error Access;
   /// SDO Abort - Object is write only
   static const SDO_Error Writeonly;
   /// SDO Abort - Object is read only
   static const SDO_Error Readonly;
   /// SDO Abort - Bad object specified
   static const SDO_Error Bad_object;
   /// SDO Abort - PDO Mapping error
   static const SDO_Error Pdo_map;
   /// SDO Abort - PDO Length error
   static const SDO_Error Pdo_length;
   /// SDO Abort - Bad parameter
   static const SDO_Error Bad_param;
   /// SDO Abort - Incompatible error
   static const SDO_Error Incompatible;
   /// SDO Abort - Hardware error
   static const SDO_Error Hardware;
   /// SDO Abort - Bad length specified
   static const SDO_Error Bad_length;
   /// SDO Abort - Data too long for object
   static const SDO_Error Too_long;
   /// SDO Abort - Data too short for object
   static const SDO_Error Too_short;
   /// SDO Abort - Subindex is invalid
   static const SDO_Error Subindex;
   /// SDO Abort - Parameter range error
   static const SDO_Error Param_range;
   /// SDO Abort - Parameter too high
   static const SDO_Error Param_high;
   /// SDO Abort - Parameter too low
   static const SDO_Error Param_low;
   /// SDO Abort - Max less then min
   static const SDO_Error Min_max;
   /// SDO Abort - General error
   static const SDO_Error General;
   /// SDO Abort - Transfer error
   static const SDO_Error Transfer;
   /// SDO Abort - Local transfer error
   static const SDO_Error Transfer_Local;
   /// SDO Abort - Transfer state error
   static const SDO_Error Transfer_State;
   /// SDO Abort - Object dictionary generation failure
   static const SDO_Error OD_Gen_Fail;
   /// SDO Abort - Unknown abort code
   static const SDO_Error Unknown;

protected:
   /// Standard protected constructor
   SDO_Error( uint16 id, const char *desc ): CanOpenError( id, desc ){}
};

/***************************************************************************/
/**
CANopen Service Data Object (SDO).  This class represents the state of a
CANopen SDO object.  SDO objects are used to access the values in the object
dictionary on a node.  This class handles the low level protocol details of
an SDO connection.
*/
/***************************************************************************/
class SDO: public Receiver
{
   /// Private copy constructor (not supported)
   SDO( const SDO& );

   /// Private assignment operator (not supported)
   SDO& operator=( const SDO& );
public:
   SDO(){}

   /// Virtual destructor
   virtual ~SDO(){}

   SDO( CanOpen &canOpen, uint32 xmit, uint32 recv, int32 to=2000 );

   // 2015.7.8 苏满佳把 to=2000 改成 to=10，为了实现模块自动搜索，否则搜索时间太长。
   const Error *Init( CanOpen &canOpen, uint32 xmit, uint32 recv, int32 to=10 );	

   const Error *Download( int16 index, int16 sub, int32 size, byte *data );
   const Error *Upload( int16 index, int16 sub, int32 &size, byte *data );

   const Error *BlockDnld( int16 index, int16 sub, int32 size, byte *data );
   const Error *BlockUpld( int16 index, int16 sub, int32 &size, byte *data );

   const Error *Dnld32( int16 index, int16 sub, uint32 data );
   const Error *Upld32( int16 index, int16 sub, uint32 &data );
   const Error *Dnld16( int16 index, int16 sub, uint16 data );
   const Error *Upld16( int16 index, int16 sub, uint16 &data );
   const Error *Dnld8( int16 index, int16 sub, uint8 data );
   const Error *Upld8( int16 index, int16 sub, uint8 &data );
   const Error *DnldString( int16 index, int16 sub, char *data );
   const Error *UpldString( int16 index, int16 sub, int32 &len, char *data );

   /// Download data using this SDO.  The passed array of data is downloaded to the
   /// object dictionary of a node on the CANopen network using this SDO.
   /// @param index The index of the object to be downloaded.
   /// @param sub The sub-index of the object to be downloaded.
   /// @param size The number of bytes of data to be downloaded
   /// @param data A character array holding the data to be downloaded.
   /// @return A valid CANopen error object.
   const Error *Download( int16 index, int16 sub, int32 size, char *data )
   {
      return Download( index, sub, size, (byte*)data );
   }

   /// Upload data using this SDO.  The value of the object is uploaded from the
   /// object dictionary of a node on the CANopen network using this SDO.  The 
   /// results of the upload are stored in the passed buffer.
   /// 
   /// @param index The index of the object to be uploaded.
   /// @param sub The sub-index of the object to be uploaded.
   /// @param size The number of bytes of data to be uploaded
   /// @param data A character array which will store the uploaded data.
   /// @return A valid CANopen error object.
   const Error *Upload( int16 index, int16 sub, int32 &size, char *data )
   {
      return Upload( index, sub, size, (byte*)data );
   }

   /// Download a 32-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be downloaded
   /// @return A valid CANopen error code.
   const Error *Dnld32( int16 index, int16 sub, int32 data ){
      return Dnld32( index, sub, (uint32)data );
   }

   /// Upload a 32-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be uploaded
   /// @return A valid CANopen error code.
   const Error *Upld32( int16 index, int16 sub, int32 &data ){
      return Upld32( index, sub, (uint32&)data );
   }

#ifdef CML_ALLOW_FLOATING_POINT
   /// Download a floating point value using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be downloaded
   /// @return A valid CANopen error code.
   const Error *DnldFlt( int16 index, int16 sub, float data ){
      void *ptr = &data;
      return Dnld32( index, sub, *( (uint32*)ptr ) );
   }

   /// Upload a floating point value using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be uploaded
   /// @return A valid CANopen error code.
   const Error *UpldFlt( int16 index, int16 sub, float &data ){
      return Upld32( index, sub, (uint32&)data );
   }
#endif

   /// Download a 16-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be downloaded
   /// @return A valid CANopen error code.
   const Error *Dnld16( int16 index, int16 sub, int16 data ){
      return Dnld16( index, sub, (uint16)data );
   }

   /// Upload a 16-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be uploaded
   /// @return A valid CANopen error code.
   const Error *Upld16( int16 index, int16 sub, int16 &data ){
      return Upld16( index, sub, (uint16 &)data );
   }

   /// Download an 8-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be downloaded
   /// @return A valid CANopen error code.
   const Error *Dnld8( int16 index, int16 sub, int8 data ){
      return Dnld8( index, sub, (byte)data );
   }

   /// Upload an 8-bit signed integer using this SDO.
   /// @param index The index of the object to access
   /// @param sub The sub-index of the object 
   /// @param data The data to be uploaded
   /// @return A valid CANopen error code.
   const Error *Upld8( int16 index, int16 sub, int8 &data ){
      return Upld8( index, sub, (byte &)data );
   }

   virtual int NewFrame( CanFrame &frame );

   /// Set the timeout used with this SDO
   /// @param to The timeout in milliseconds
   void SetTimeout( int32 to ){ timeout = to; }

   /// Get the timeout used with this SDO
   /// @return The timeout in milliseconds
   int32 GetTimeout( void ){ return timeout; }

   /// Return the transmit ID associated with the SDO
   /// @return The transmit CAN ID
   uint32 getXmitID( void ){ return xmitID; }

   /// If true, the SDO will attempt to use a block upload when 
   /// large amounts of data need to be passed.
   /// This will be set to false if the block upload fails.
   ///
   /// Default: false
   bool blkUpldOK;

   /// If true, the SDO will attempt to use a block download when 
   /// large amounts of data need to be passed.
   /// This will be set to false if the block download fails.
   ///
   /// Default: false
   bool blkDnldOK;

private:

   void SendAbort( int32 code );
   const Error *getAbortRcvdErr( CanFrame &frame );

   const Error *WaitForTransfer( int32 &size );

   /// Mutex used to control access to the SDO object
   Mutex mutex;

   /// Semaphore used to wait on state changes
   Semaphore sem;

   /// SDO state variable
   int16 state;

   /// SDO Transmit message COB ID
   uint32 xmitID;

   /// These three bytes hold the object multiplexor 
   /// (index/subindex) being accessed by the SDO
   byte mplex[3];

   /// Points to the last error that occurred
   const Error *lastErr;

   /// Points to a buffer that holds data to be up/downloaded
   byte *dataPtr;

   /// Timeout value (ms) for SDO messages
   int32 timeout;

   /// This variable counts the bytes received/transmitted.
   int32 count;

   /// Gives the number of bytes of data left to up/download
   int32 remain;

   /// This variable tracks the value of the toggle bit in up/downloads
   char toggle;

   /// For block transfers, this gives the last sequence number received
   uint8 lastBlkSeq;

   /// For block transfers, size of a block in units of segments.
   uint8 blockSize;
};

CML_NAMESPACE_END()

#endif

