/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file holds code for the top level CANopen class.
This class is used for over all control of the CANopen network.

*/

#include "CML.h"

CML_NAMESPACE_USE();

// static CANopen error objects
CML_NEW_ERROR( CanOpenError, ThreadStart,     "Error starting CANopen read thread" );
CML_NEW_ERROR( CanOpenError, BadParam,        "Bad parameter value" );
CML_NEW_ERROR( CanOpenError, SDO_Busy,        "SDO object is not idle" );
CML_NEW_ERROR( CanOpenError, SDO_Timeout,     "Timeout waiting on SDO" );
CML_NEW_ERROR( CanOpenError, SDO_Unknown,     "Unknown SDO error" );
CML_NEW_ERROR( CanOpenError, SDO_BadMuxRcvd,  "Invalid multiplexor received in SDO message" );
CML_NEW_ERROR( CanOpenError, SDO_BadMsgRcvd,  "Invalid format for received SDO message" );
CML_NEW_ERROR( CanOpenError, BadNodeID,       "The specified Node ID was invalid" );
CML_NEW_ERROR( CanOpenError, NotInitialized,  "The referenced object has not been initialized" );
CML_NEW_ERROR( CanOpenError, Initialized,     "The referenced object is already initialized" );
CML_NEW_ERROR( CanOpenError, NotSupported,    "The feature is not supported" );
CML_NEW_ERROR( CanOpenError, MonitorRunning,  "Heartbeat or node guarding already started" );
CML_NEW_ERROR( CanOpenError, IllegalFieldCt,  "Illegal field count returned for the requested object" );
CML_NEW_ERROR( CanOpenError, RcvrNotFound,    "No enabled receiver could be found for that ID" );
CML_NEW_ERROR( CanOpenError, RcvrPresent,     "A CAN receiver using that ID is already enabled" );
CML_NEW_ERROR( CanOpenError, Closed,          "The CANopen port is closed" );

/***************************************************************************/
/**
Default constructor.  Simply initializes some local variables.
*/
/***************************************************************************/
CanOpen::CanOpen( void )
{
   synchProducer = 0;
   errorFrameCt = 0;

   for( int i=0; i<CML_HASH_SIZE; i++ )
      hash[i] = 0;
   can = 0;
}

/***************************************************************************/
/**
CanOpen Destructor.  This closes the CANopen network.
*/
/***************************************************************************/
CanOpen::~CanOpen( void )
{
   Close();
}

/***************************************************************************/
/**
Close the CANopen network.  This disables all receivers and stops the
thread that listens on the CAN network.
*/
/***************************************************************************/
void CanOpen::Close( void )
{
   for( int i=0; i<CML_HASH_SIZE; i++ )
   {
      if( hash[i] )
      {
	 DisableReceiver( hash[i] );
	 hash[i] = 0;
      }
   }
   stop();

   if( can )
   {
      can->canOpenPtr = 0;
      can->Close();
      can = 0;
   }
}

/***************************************************************************/
/**
Open the CANopen network.  This function performs the one time initialization 
necessary to communication via the CANopen network.  It should be the first 
function called for the CANopen object.

All configurable settings will be set to their defaults when the CanOpen object
is opened using this method.  For a list of CanOpen object settings and their
default values, please see the CanOpenSettings object.

@param canRef A reference to the CAN interface object that will be used for
       all low level communication over the network.
		 
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::Open( CanInterface &canRef )
{
   CanOpenSettings settings;
   return Open( canRef, settings );
}

/***************************************************************************/
/**
Open the CANopen network.  This function performs the one time initialization 
necessary to communication via the CANopen network.  It should be the first 
function called for the CANopen object.

This version of the Open function takes a CanOpenSettings object reference
as it's second parameter.  The data members of the settings object may be
used to configure some of the CanOpen object's behavior.

@param canRef A reference to the CAN interface object that will be used for
       all low level communication over the network.
		 
@param settings A reference to a CanOpenSettings object.  This object is used
       to customize the behavior of the CanOpen object.
		 
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::Open( CanInterface &canRef, CanOpenSettings &settings )
{
   if( can )
      return &CanOpenError::Initialized;

   // Keep a pointer to the CAN interface object.
   can = &canRef;

   // Open the low level CAN interface
   const Error *err = can->Open();

   if( err && (err != &CanError::AlreadyOpen) )
      return err;

   // Save a pointer to this object in the CanInterface object.
   // This is used to ensure clean destruction of both objects.
   can->canOpenPtr = this;

   setPriority( settings.readThreadPriority );

   // Start a thread that will listen for messages 
   // on the CAN network.
   if( start() )
      return &CanOpenError::ThreadStart;

   return 0;
}

/***************************************************************************/
/**
Send a Network Management message to one or more nodes on the network.
@param code The network management function code to be sent.
@param nodeID The ID of the node to send the message to.  Use 0 to broadcast
       the message to all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::NMT_Msg( int code, int nodeID )
{
   // Check for a valid Node ID
   if( nodeID < 0 || nodeID > 127 )
      return &CanOpenError::BadNodeID;

   CanFrame frame;
   frame.id = 0;
   frame.type = CAN_FRAME_DATA;
   frame.length = 2;
   frame.data[0] = code;
   frame.data[1] = nodeID;

   return Xmit( frame );
}

/***************************************************************************/
/**
Send a network management message to start the specified node.  All nodes
are started if the passed node ID is zero.
@param nodeID The ID of the node to started, or zero for all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::StartNode( int nodeID ){ return NMT_Msg( 1, nodeID ); }

/***************************************************************************/
/**
Send a network management message to stop the specified node.  All nodes
are stopped if the passed node ID is zero.
@param nodeID The ID of the node to stop, or zero for all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::StopNode( int nodeID ) { return NMT_Msg( 2, nodeID ); }

/***************************************************************************/
/**
Send a network management message to put the specified node in pre-operational
state.  All nodes are made pre-operational if the passed node ID is zero.
@param nodeID The ID of the node to make pre-operational.  Zero for all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::PreOpNode( int nodeID ){ return NMT_Msg( 128, nodeID ); }

/***************************************************************************/
/**
Send a network management message to reset the specified node.  All nodes
are reset if the passed node ID is zero.
@param nodeID The ID of the node to reset, or zero for all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::ResetNode( int nodeID ){ return NMT_Msg( 129, nodeID ); }

/***************************************************************************/
/**
Send a network management message to reset the communications of the specified 
node.  All nodes have their communications reset if the passed node ID is zero.
@param nodeID The ID of the node to reset, or zero for all nodes.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::ResetComm( int nodeID ){ return NMT_Msg( 130, nodeID ); }

/***************************************************************************/
/**
Transmit a frame over the CANopen network.
*/
/***************************************************************************/
const Error *CanOpen::Xmit( CanFrame &frame, int32 timeout )
{
   if( !can ) return &CanOpenError::Closed;
   return can->Xmit( frame, timeout );
}

/***************************************************************************/
/**
CAN network read thread.  This function defines the thread that will be
used to read the CAN network and pass received frames to the various 
CANopen network reader objects.
*/
/***************************************************************************/
void CanOpen::run( void )
{
   Receiver *r;
   CanFrame frame;

   while( 1 )
   {
      // If the port is closed, return.  This will stop the thread.
      if( !can ) 
	 return;

      const Error *err = can->Recv( frame );

      if( err )
      {
	 cml.Error( "Error reading from CAN interface: %s\n", err->toString() );
	 sleep( 5 );
	 continue;
      }

      if( frame.type == CAN_FRAME_ERROR )
      {
	 errorFrameCt++;
	 continue;
      }

      hashMutex.Lock();

      r = *searchHash( frame.id );
      if( r )
	 r->NewFrame(frame);

      hashMutex.Unlock();
   }
}

/***************************************************************************/
/**
Default constructor for CanOpenSettings object.
This constructor simply sets all the settings to their default values.
*/
/***************************************************************************/
CanOpenSettings::CanOpenSettings( void )
{
   readThreadPriority = 9;
}

/***************************************************************************/
/**
Default constructor for a CANopen receiver object.
*/
/***************************************************************************/
Receiver::Receiver( void )
{
   // Whenever the receiver is disabled, we self reference with the 
   // next pointer.  This is a quick way to identify an enabled receiver
   next = this;
   co = 0;
}

/***************************************************************************/
/**
Initialize a new CANopen receiver.  After construction, the receiver will 
be disabled.  Call Enable() to allow messages to be received.
@param canOpen Reference to the CANopen network object that this receiver
       is associated with.
@param id COB ID of the receive message
*/
/***************************************************************************/
Receiver::Receiver( CanOpen &canOpen, uint32 id )
{
   next = this;
   co = 0;
   Init( canOpen, id );
}

/***************************************************************************/
/**
Destructor for CANopen receiver objects.  This destructor ensures that the
receiver is disabled before it is destroyed.
*/
/***************************************************************************/
Receiver::~Receiver()
{
   UnInit();
}

/***************************************************************************/
/**
Initialize the CANopen receiver object.  This function should be called 
before the receiver is used in any way.  It can only be called once.
@param canOpen Reference to the CANopen network object that this receiver
       is associated with.
@param id COB ID of the receive message
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Receiver::Init( CanOpen &canOpen, uint32 id )
{
   const Error *err = CanInterface::ChkID( id );
   if( err ) return err;

   DisableReceiver();

   this->id = id;
   co = &canOpen;
   return 0;
}

/***************************************************************************/
/**
Un-initialize a CANopen receiver.  This returns the receiver object to it's
uninitialized state.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Receiver::UnInit( void )
{
   DisableReceiver();
   co = 0;
   return 0;
}

/***************************************************************************/
/**
Process a new received CAN bus frame.  This virtual function is called by
the CANopen read thread every time a CAN frame is received over the network
with an ID matching the receivers ID if the receiver is enabled.  

Note that this function is called from the CANopen receive thread.  No other
receive frames will be processed until this function returns.

Also note that the map object used to associate message IDs with receive objects
is locked when this function is called.  The locking is required to prevent a 
race condition that could occur when a receive object is disabled and it's 
memory is deallocated.  Since the map is locked, it's illegal to Enable() or
Disable() any receive object from within this function.

@param frame The CAN frame to be processed.  Note that the memory holding the
       frame structure may be reused after the call returns.  If the frame 
		 contents are to be used after the return the a copy of the frame should
		 be made. 
		 
@return non-zero if the frame was handled, zero if the frame type was unknown.
*/
/***************************************************************************/
int Receiver::NewFrame( CanFrame &frame )
{
   return 0;
}

/***************************************************************************/
/**
Return the receive COB ID associated with this CANopen message receiver.
@return The receive COB ID.
*/
/***************************************************************************/
uint32 Receiver::getRecvID( void )
{
   return id;
}

/***************************************************************************/
/**
Enable reception of this message type.  Once enabled, any messages received
over the CANopen network of this type will be passed to the Receiver
using the member function NewFrame.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Receiver::EnableReceiver()
{
   if( !co ) return &CanOpenError::NotInitialized;

   return co->EnableReceiver( this );
}

/***************************************************************************/
/**
Search a hash table of receiver objects associated with this network.
The CanOpen object maintains a hash of all enabled receivers on the 
network.  Each time a new CAN frame is received, the hash is searched for
a receiver matching the frame's ID.  This function handles the details of
the search.

This function is also used when adding receivers to the tree.  To aid this
use, it actually returns a pointer to a pointer to the receiver, or to the
place where the new receiver would be added.

Note, it's assumed that the hash mutex will be locked when this function 
is called.

@param id The CAN message ID of the receiver to be found.
@return A pointer to a pointer to the receiver, or the location where the
        receiver should be added if there is none with a matching ID.
*/
/***************************************************************************/
Receiver **CanOpen::searchHash( uint32 id )
{
   int i = id % CML_HASH_SIZE;

   Receiver **pp = &hash[i];

   while( 1 )
   {
      if( !*pp )
	 return pp;

      uint32 x = (*pp)->id;

      if( id == x )
	 return pp;

      pp = &( (*pp)->next);
   }
}

/***************************************************************************/
/**
Enable reception handling of the message identified by this Receiver
object.  The receiver is enabled by adding it to a binary tree of 
receiver objects maintained by the CanOpen object.

@param rPtr Pointer to the receiver to add
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::EnableReceiver( Receiver *rPtr )
{
   const Error *err = 0;

   hashMutex.Lock();

   Receiver **pp = searchHash( rPtr->id );

   if( *pp )
      err = &CanOpenError::RcvrPresent;
   else
   {
      rPtr->next = 0;
      *pp = rPtr;
   }

   hashMutex.Unlock();

   return err;
}

/***************************************************************************/
/**
Disable reception handling of the message identified by this Receiver
object.  Receivers are disabled by removing them from a binary tree of 
receiver objects maintained by the CanOpen object.

@param rPtr A pointer to the receiver to disable.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *CanOpen::DisableReceiver( Receiver *rPtr )
{
   const Error *err = 0;

   hashMutex.Lock();

   Receiver **pp = searchHash( rPtr->id );

   if( !*pp )
      err = &CanOpenError::RcvrNotFound;

   else
   {
      rPtr = *pp;
      *pp = rPtr->next;

      // Point the receiver back at itself.
      // This allows the receiver to determine it's disabled.
      rPtr->next = rPtr;
   }

   hashMutex.Unlock();
   return err;
}

/***************************************************************************/
/**
Disable reception of this message type.  Any CAN frames received by the
CANopen object of a disabled message type will be ignored by the CANopen
object.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Receiver::DisableReceiver()
{
   if( !co ) return &CanOpenError::NotInitialized;

   if( next == this ) return &CanOpenError::RcvrNotFound;

   return co->DisableReceiver( this );
}

