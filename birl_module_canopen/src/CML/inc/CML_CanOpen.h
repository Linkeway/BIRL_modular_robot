/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This header file defines the classes used for the top level of 
the CANopen network.  

*/

#ifndef _DEF_INC_CANOPEN
#define _DEF_INC_CANOPEN

#include "CML_Settings.h"
#include "CML_Can.h"
#include "CML_Error.h"
#include "CML_Threads.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/**
This class holds the error codes that describe CANopen 
error conditions.
*/
class CanOpenError: public CanError
{
public:
   /// Indicates that the specified port name is invalid
   static const CanOpenError ThreadStart;

   /// One of the parameters passed to the CANopen function is invalid
   static const CanOpenError BadParam;

   /// The SDO object is busy
   static const CanOpenError SDO_Busy;

   /// The SDO up/download failed with a timeout
   static const CanOpenError SDO_Timeout;

   /// Some unknown error occurred
   static const CanOpenError SDO_Unknown;

   /// The mux (index/sub-index) received in a SDO message
   /// is inconsistent with the object being accessed.
   static const CanOpenError SDO_BadMuxRcvd;

   /// An improperly formatted SDO message was received.
   static const CanOpenError SDO_BadMsgRcvd;

   /// An illegal node ID was specified
   static const CanOpenError BadNodeID;

   /// The object being used has not been initialized.
   /// This error indicates a coding error, i.e. trying
   /// to use a Receiver object without initializing it.
   static const CanOpenError NotInitialized;

   /// An attempt was made to initialize an object that
   /// has already been initialized, and doesn't allow
   /// multiple initialization.
   static const CanOpenError Initialized;

   /// The requested feature is not supported by this node
   static const CanOpenError NotSupported;

   /// Monitor already running - An attempt is made to start
   /// the heartbeat or node guarding and it's already running.
   static const CanOpenError MonitorRunning;

   /// The node returned an illegal field count for the object
   /// being requested in it's object dictionary.
   static const CanOpenError IllegalFieldCt;

   /// An attempt was made to disable a receiver that wasn't enabled
   static const CanOpenError RcvrNotFound;

   /// An attempt was made to enable a receiver that was already enabled
   static const CanOpenError RcvrPresent;

   /// The CANopen port is closed
   static const CanOpenError Closed;

protected:
   CanOpenError( uint16 id, const char *desc ): CanError( id, desc ){}
};

/***************************************************************************/
/**
Configuration object used to customize global settings for the CANopen
network.  An object of this type may be passed to the CanOpen::Open()
method when the network is first opened.

If no CanOpenSettings object is passed to the CanOpen::Open() method, 
then the behavior is exactly the same as passing a CanOpenSettings object
with the default settings.
*/
/***************************************************************************/
class CanOpenSettings
{
public:
   CanOpenSettings();

   /// Defines the read thread priority.  The read thread is started
   /// when the CanOpen object is first opened (using CanOpen::Open()).
   /// This thread is responsible for reading messages from the
   /// CANopen network and calling the message handlers associated with
   /// them.  It should be run at a relatively high priority.
   /// Default: 9
   int readThreadPriority;
};

/***************************************************************************/
/**
The CanOpen class is the top level interface into the CANopen network.
There should be at least one object of this class in every CANopen based
application.  Normally, only one object will be necessary, however if more
then one independent CANopen network is in use, then more then one object
will be necessary.

On startup, a low level CAN interface object should be created.  This 
object should be passed to the CANopen object's Open() method.
*/
/***************************************************************************/
class CanOpen: public Thread
{
   /// Private copy constructor (not supported)
   CanOpen( const CanOpen& );

   /// Private assignment operator (not supported)
   CanOpen& operator=( const CanOpen& );

public:
   CanOpen( void );
   virtual ~CanOpen( void );
   const Error *Open( CanInterface &can );
   const Error *Open( CanInterface &can, CanOpenSettings &settings );
   void Close( void );

   const Error *Xmit( CanFrame &frame, int32 timeout=2000 );
   const Error *ResetNode( int nodeID=0 );
   const Error *ResetComm( int nodeID=0 );
   const Error *PreOpNode( int nodeID=0 );
   const Error *StartNode( int nodeID=0 );
   const Error *StopNode( int nodeID=0 );

   /// Return the node ID of the synch producer for this network.
   /// @return The synch producer node ID, or 0 if no synch producer
   /// has been registered.
   int16 GetSynchProducer( void ){
      return synchProducer;
   }

   /// Set the node ID of the synch producer for this network.
   /// @param nodeID The new synch producer node ID, or 0 for none.
   void SetSynchProducer( int16 nodeID ){
      synchProducer = nodeID;
   }

   /// Return the number of error frames received over then CAN network
   /// since the last time the counter was cleared
   /// @return The number of error frames received since the last call
   /// to CanOpen::ClearErrorFrameCounter();
   int32 GetErrorFrameCounter( void ){
      return errorFrameCt;
   }

   /// Clear the error frame counter
   void ClearErrorFrameCounter( void ){
      errorFrameCt = 0;
   }

private:
   const Error *NMT_Msg( int code, int nodeID );

   const Error *EnableReceiver( class Receiver *receiver );
   const Error *DisableReceiver( class Receiver *receiver );
   Receiver **searchHash( uint32 id );

   /// This hash is used to keep track of the Receiver
   /// objects that are enabled.
   class Receiver *hash[ CML_HASH_SIZE ];

   /// This mutex is used to control access to the receiver hash.
   Mutex hashMutex;

   /// Pointer to the can interface object passed during construction
   CanInterface *can;

   /// This variable is used to determine if a synch message producer
   /// has been assigned to this network.
   int16 synchProducer;

   /// This variable is used to keep a count of CAN error frames for
   /// diagnostic purposes.
   int32 errorFrameCt;

   void run( void );
   friend class Receiver;
   friend class CanInterface;
};

/***************************************************************************/
/**
CANopen receiver object.  This class allows the programmer to create routines
that are run whenever a CAN frame with a specific ID is received.  

To run code when a message is received, create a new class that extends
Receiver.  The Receiver::Init() function should be called with the COB-ID 
of the frames to be received.  Whenever this new class is Enabled(), the
member function NewFrame() will be called once for every frame received with
a matching ID.
*/
/***************************************************************************/
class Receiver
{
public:
   Receiver();
   Receiver( CanOpen &canOpen, uint32 id );
   virtual ~Receiver();

   const Error *Init( CanOpen &canOpen, uint32 id );
   const Error *UnInit( void );

   virtual int NewFrame( CanFrame &frame );
   uint32 getRecvID( void );

   const Error *EnableReceiver();
   const Error *DisableReceiver();

   /// Return a reference to the CanOpen object 
   /// associated with this receiver
   /// @return The CanOpen object 
   CanOpen &GetCanOpen(void){
      return *co;
   }

   /// Return true if this receiver has been initialized
   /// @return True if initialized
   bool IsInitialized( void )
   {
      return co != 0;
   }

protected:
   CanOpen *co;

private:
   uint32 id;
   Receiver *next;
   friend class CanOpen;

   /// Private copy constructor (not supported)
   Receiver( const Receiver& );

   /// Private assignment operator (not supported)
   Receiver& operator=( const Receiver& );
};

/***************************************************************************/
/**
CANopen Layer Setting Services object.

The Layer Setting Services (LSS) protocol is part of the CANopen network 
standard.  The intent of LSS is to allow low level network settings, such
as the network bit rate and device node ID numbers to be configured over 
the network.

The CANopen protocol requires each device on the network to have a unique
node ID number in the range 1 to 127.  In general, it's not possible to 
communicate with a device using CANopen if it doesn't have a unique node
ID in this range.

The LSS protocol allows some limited communication with any device on the 
network even if it doesn't have a node ID set.  This allows node ID numbers
to be assigned to devices over the network.

This object implements the LSS protocol and allows devices on the network
to be queried and configured.  

For more detailed information on the LSS protocol please see the CANopen 
standard document DSP305.
*/
/***************************************************************************/
class LSS: public Receiver
{
   Semaphore sem;
   int32 timeout;
   int max, tot;
   uint8 recvCS;
   uint32 *serial;
   uint32 recvData;
public:
   LSS( CanOpen &co );
   int FindAmplifiers( int max, uint32 serial[] );

   /// Set the timeout value used by the LSS protocol.
   /// @param to The new timeout (milliseconds)
   void setTimeout( int32 to ){ timeout = to; }

   /// Get the current timeout value used by the LSS protocol
   /// @return The current timeout in milliseconds.
   int32 getTimeout( void ){ return timeout; }

   const Error *GetAmpNodeID( uint32 serial, byte &nodeID );
   const Error *SetAmpNodeID( uint32 serial, byte nodeID );
protected:
   const Error *SelectAmp( uint32 serial );
   uint32 FindAmpSerial( uint32 low, uint32 high );
   int NewFrame( CanFrame &frame );
   const Error *Xmit( byte cs, uint32 data=0 );
private:
   /// Private copy constructor (not supported)
   LSS( const LSS& );

   /// Private assignment operator (not supported)
   LSS& operator=( const LSS& );
};

CML_NAMESPACE_END()

#endif

