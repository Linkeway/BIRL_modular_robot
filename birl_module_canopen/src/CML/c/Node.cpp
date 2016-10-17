/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file holds code to implement the CANopen node related objects.
*/

#include "CML.h"

CML_NAMESPACE_USE();

/**************************************************
* Node Error objects
**************************************************/
CML_NEW_ERROR( NodeError, GuardTimeout, "Node guarding timeout" );

/**************************************************
* Bits used with the node guarding event map
**************************************************/
#define GUARD_EVENT_MSG_RVCD     0x00000001
#define GUARD_EVENT_CHANGE       0x00000002
#define GUARD_EVENT_DSRDSTATE    0x00000004

/***************************************************************************/
/**
Initialize a CANopen node emergency receiver.  This function should be called
once and only once for each emergency receiver.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *NodeEmcyRcvr::Init( Node *nodePtr )
{
   // Return an error if the object has already been initialized.
   if( IsInitialized() ) return &CanOpenError::Initialized;

   // Init the object
   node = nodePtr;

   return Receiver::Init( *(node->co), 0x80 + node->nodeID );
}

/***************************************************************************/
/**
Handle an node's emergency object.  The HandleEmergency() member function 
of the node that this emergency handler is associated with is called.
@param frame The emergency frame
@return Always returns 1.
*/
/***************************************************************************/
int NodeEmcyRcvr::NewFrame( CanFrame &frame )
{
   node->HandleEmergency( frame );
   return 1;
}

/***************************************************************************/
/**
Default CANopen node object constructor.  This constructor simple marks the
object as uninitialized.  The Init() function must be called before this
object can be used.
*/
/***************************************************************************/
Node::Node()
{
}

/***************************************************************************/
/**
Initialize the CANopen Node object.
@param canOpen The CANopen network object that this node is associated with.
@param nodeID The node's ID.  This must range from 1 to 127 for the node to
              be successfully initialized.
*/
/***************************************************************************/
Node::Node( CanOpen &canOpen, int16 nodeID )
{
   Init( canOpen, nodeID );
}

/***************************************************************************/
/**
CANopen node destructor.
*/
/***************************************************************************/
Node::~Node()
{
   cml.Debug( "Node: %d destroyed\n", nodeID );
   UnInit();
}

/***************************************************************************/
/**
Initialize the CANopen Node object.  Note that a CANopen node object must
be initialized once and only once.  This function should be used to initialize
the object if it was created using the default constructor.

@param canOpen The CANopen network object that this node is associated with.
@param nodeID The node's ID.  This must range from 1 to 127 for the node to
              be successfully initialized.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::Init( CanOpen &canOpen, int16 nodeID )
{
   const Error *err;

   // Check for invalid node ID
   if( nodeID < 1 || nodeID > 127 )
      return &CanOpenError::BadNodeID;

   // Un-initialize first if necessary
   if( IsInitialized() && (err = UnInit()) != 0 )
      return err;

   // Init some local parameters
   this->nodeID = nodeID;
   state = NODESTATE_UNKNOWN;

   // Start out with node guarding disabled, and 
   // start the guard monitor thread.
   guardEvents.clrBits( 0xFFFFFFFF );
   guardTimeout = -1;
   guardToggle = -1;
   guardType = GUARDTYPE_NONE;
   desired = NODESTATE_INVALID;
   err = Thread::start();
   if( err ) return err;

   // Enable reception of node guarding messages
   err = Receiver::Init( canOpen, 0x700+nodeID );
   if( err ) return err;
   Receiver::EnableReceiver();

   // Init my SDO
   err = sdo.Init( canOpen, 0x600+nodeID, 0x580+nodeID );
   if( err ) return err;

   // Init the emergency object handler
   err = emcy.Init( this );
   if( err ) return err;

   // Enable the emergency receiver
   emcy.EnableReceiver();

   return 0;
}

/***************************************************************************/
/**
Un-initialize the Node object.  This puts the object back to it's default
state.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *Node::UnInit( void )
{
   const Error *err = stop();

   if( !err ) err = Receiver::UnInit();
   if( !err ) err = sdo.UnInit();
   if( !err ) err = emcy.UnInit();
   return err;
}

/***************************************************************************/
/**
Associate the passed PDO object with this node.  The PDO
will be setup as this node's nth PDO.

@param slot Which PDO slot to assign this PDO to.
@param pdo The PDO object.
@param enable If true, the PDO will be enabled after being setup (default).
              If false, the PDO will be setup but not enabled.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::PdoSet( uint16 slot, PDO &pdo, bool enable )
{
   // Make sure the slot is reasonable.
   if( slot > 511 )
      return &CanOpenError::BadParam;

   uint16 base = slot+pdo.BaseIndex();

   // Upload the current PDO ID from the
   // amplifier.  If the PDO is enabled, 
   // disable it now.
   uint32 pdoID;
   const Error *err = sdo.Upld32( base, 1, pdoID );
   if( err ) return err;

   if( !(pdoID & 0x80000000) )
   {
      pdoID |= 0x80000000;
      err = sdo.Dnld32( base, 1, pdoID );
      if( err ) return err;
   }

   // Find the ID code for this PDO
   uint32 newID = pdo.GetID() | 0x80000000;
   if( !pdo.GetRtrOk() ) newID |= 0x40000000;

   // Update the PDO ID if it's different.
   if( newID != pdoID )
   {
      err = sdo.Dnld32( base, 1, newID );
      if( err ) return err;
   }

   // Compare the new and old PDO type.
   // If different, update it.
   byte oldType;
   err = sdo.Upld8( base, 2, oldType );
   if( err ) return err;

   if( oldType != pdo.GetType() )
   {
      err = sdo.Dnld8( base, 2, pdo.GetType() );
      if( err ) return err;
   }

   // Update inhibit time & event time (not supported yet)

   // Get the PDO variable mapping info
   uint32 codes[PDO_MAP_LEN];
   byte ct = pdo.GetMapCodes( codes );

   base += 0x200;

   // Compare the new mapping info to the info already
   // in the PDO.  If it's different, then I'll update it.
   byte pdoCt;
   err = sdo.Upld8( base, 0, pdoCt );
   if( err ) return err;

   bool done = false;

   if( pdoCt == ct )
   {
      uint32 var;
      byte i;
      for( i=0; i<ct; i++ )
      {
	 err = sdo.Upld32( base, i+1, var );
	 if( err ) return err;

	 if( var != codes[i] )
	    break;
      }
      if( i == ct )
	 done = true;
   }

   if( !done )
   {
      // Clear out any old mapping
      err = sdo.Dnld8( base, 0, (byte)0 );
      if( err ) return err;

      // Download the new mapping
      for( byte i=0; i<ct; i++ )
      {
	 err = sdo.Dnld32( base, i+1, codes[i] );
	 if( err ) return err;
      }

      // Active the new mapping
      err = sdo.Dnld8( base, 0, ct );
      if( err ) return err;
   }

   // Enable the PDO if so requested
   if( enable )
      return PdoEnable( slot, pdo );

   return 0;
}

/***************************************************************************/
/**
Enable the specified PDO.  When this function is called it is assumed
that the PDO has already be setup.
@param n The slot number of the PDO to enable
@param base The index number in the object dictionary where this type
       of PDO starts (0x1800 for transmit, 0x1400 for receive)
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::PdoEnable( uint16 n, uint16 base )
{
   // Make sure the slot is reasonable.
   if( n > 511 )
      return &CanOpenError::BadParam;

   n += base;

   // Upload the PDO's ID
   uint32 id;
   const Error *err = sdo.Upld32( n, 1, id );
   if( err ) return err;

   // If it's not yet enabled, enable it
   if( id & 0x80000000 )
      err = sdo.Dnld32( n, 1, id&0x7fffffff );

   return err;
}

/***************************************************************************/
/**
Disable the specified PDO.
@param n The slot number of the transmit PDO to disable
@param base The index number in the object dictionary where this type
       of PDO starts (0x1800 for transmit, 0x1400 for receive)
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::PdoDisable( uint16 n, uint16 base )
{
   // Make sure the slot is reasonable.
   if( n > 511 )
      return &CanOpenError::BadParam;

   n += base;

   // Upload the PDO's ID
   uint32 id;
   const Error *err = sdo.Upld32( n, 1, id );
   if( err ) return err;

   // If it's enabled, disable it
   if( !(id & 0x80000000) )
      err = sdo.Dnld32( n, 1, (uint32)(id|0x80000000) );

   return err;
}

/***************************************************************************/
/**
Get the error history array (CANopen object 0x1003).

@param ct    When the function is first called, this variable holds the maximum number
             of errors that can be stored in the err array (i.e. the length of the array).
			    On return, the actual number of errors uploaded will be stored here.
@param array An array of 32-bit integers that will be used to return the list of errors.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::GetErrorHistory( uint16 &ct, uint32 *array )
{
   uint32 i;

   // Upload the first element in the error array.
   // This holds the actual number of errors in it's 
   // lowest byte.
   const Error *err = sdo.Upld32( 0x1003, 0, i );
   if( err ) return err;

   if( i > 254 ) i = 254;

   // Limit the number of errors to download to ct
   if( i < (uint32)ct ) ct = i;

   for( i=1; i<=(uint32)ct; i++ )
   {
      err = sdo.Upld32( 0x1003, i, array[i-1] );
      if( err ) return err;
   }

   return 0;
}

/***************************************************************************/
/**
Get the CANopen identity object for this node (object dictionary entry 0x1018).
Note that only the VendorID field is mandatory.  Any unsupported fields will
be returned as zero.

@param id The identity object to be filled in by this call
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::GetIdentity( NodeIdentity &id )
{
   // Find the number of supported fields
   int8 ct;
   const Error *err = sdo.Upld8( 0x1018, 0, ct );
   if( err ) return err;

   if( ct < 1 || ct > 4 )
      return &CanOpenError::IllegalFieldCt;

   id.productCode = 0;
   id.revision    = 0;
   id.serial      = 0;

   err = sdo.Upld32( 0x1018, 1, id.vendorID );
   if( !err && ct > 1 ) err = sdo.Upld32( 0x1018, 2, id.productCode );
   if( !err && ct > 2 ) err = sdo.Upld32( 0x1018, 3, id.revision    );
   if( !err && ct > 3 ) err = sdo.Upld32( 0x1018, 4, id.serial      );

   return err;
}

/***************************************************************************/
/**
Reset this node.
@return An error object
*/
/***************************************************************************/
const Error *Node::ResetNode( void )
{
   cml.Debug( "ResetNode %d.\n", GetNodeID() );

   // Disable the node guarding task
   mutex.Lock();
   guardType    = GUARDTYPE_NONE;
   guardTimeout = -1;
   guardToggle  = -1;
   guardEvents.setBits( GUARD_EVENT_CHANGE );
   mutex.Unlock();

   // Wait for the node guarding task to change it's state
   EventNone e = GUARD_EVENT_CHANGE;

   const Error *err = e.Wait( guardEvents, 500 );
   if( err )
   {
      cml.Warn( "Error %s waiting for node guard task during ResetNode call\n", err->toString() );
      return err;
   }

   // Send the node a reset message
   err = co->ResetNode( nodeID );
   if( err ) return err;

   // Wait for the state to change to pre-op without sending
   // a remote request.  The node should indicate that state
   // on boot up.
   return WaitStateChange( NODESTATE_PRE_OP, false );
}

/***************************************************************************/
/**
Disable node guarding & heartbeat monitoring.

@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::StopGuarding( void )
{
   // Clear both heartbeat and node guard times.
   sdo.Dnld16( 0x1017, 0, (uint16)0 );
   sdo.Dnld16( 0x100C, 0, (uint16)0 );

   mutex.Lock();
   guardType    = GUARDTYPE_NONE;
   guardTimeout = -1;
   guardToggle  = -1;
   guardEvents.setBits( GUARD_EVENT_CHANGE );
   mutex.Unlock();

   return 0;
}

/***************************************************************************/
/**
Enable heartbeat messages from this node, and start a thread to monitor
them.
@param period The producer timeout value (milliseconds).  The node will
       be configured to produce a heartbeat message at this interval.
		 
@param timeout The additional number of milliseconds that the monitor
       thread will wait before indicating an error.  Thus, the consumer
		 heartbeat interval will be (period + timeout).
		 
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::StartHeartbeat( uint16 period, uint16 timeout )
{
   // If the period is sent as zero, return an error.
   if( !period )
      return &CanOpenError::BadParam;

   // Set the node's heartbeat time.
   const Error *err = sdo.Dnld16( 0x1017, 0, period );

   if( !err )
   {
      mutex.Lock();
      guardType    = GUARDTYPE_HEARTBEAT;
      guardTimeout = (int32)period + (int32)timeout;
      guardToggle  = -1;
      guardEvents.setBits( GUARD_EVENT_CHANGE );
      mutex.Unlock();
   }

   return err;
}

/***************************************************************************/
/**
Enable node guarding on this node.

When node guarding is enabled, a new thread is created which will send a 
remote request to this node every (guardTime) milliseconds.  The node must
respond to this message within the guard time.  If the node does not respond
then the thread will notify the node of a state change.

@param guardTime The period in milliseconds of the guard messages sent
       to the node.  It can range from 1 to 65535.
		 
@param lifeFactor A multiplier used by the node to determine how long to 
       wait for a node guarding message from the host before indicating
		 a local error.  The nodes timeout (life time) is guardTime * lifeFactor.
		 This parameter must be between 0 and 255.  If it's zero, then life
		 guarding on the node is disabled.
		 
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::StartNodeGuard( uint16 guardTime, byte lifeFactor )
{
   // If guard time is zero, return an error.
   // Note that the lifeFactor can be zero, this disables the life
   // guarding inside the node.
   if( !guardTime )
      return &CanOpenError::BadParam;

   // Program the life time factor
   const Error *err;
   err = sdo.Dnld8( 0x100D, 0, lifeFactor );

   // Program the guard time
   if( !err ) err = sdo.Dnld16( 0x100C, 0, guardTime );

   // Try to set the heartbeat time to zero.
   // This may fail (since the heartbeat may not be
   // supported).
   if( !err ) sdo.Dnld16( 0x1017, 0, (uint16)0 );

   if( !err )
   {
      mutex.Lock();
      guardType    = GUARDTYPE_NODEGUARD;
      guardTimeout = guardTime;
      guardToggle  = -1;
      guardEvents.setBits( GUARD_EVENT_CHANGE );
      mutex.Unlock();
   }

   return err;
}

/***************************************************************************/
/**
Wait for the node state to change.  This function is used after sending out
a network management message intended to change the node's state.  It sends
a remote transmit request to the node and waits for the guard message to
be returned.

This process allows us to track the node's state locally without fear of 
race conditions that would arise if we simply assumed that sending out the
NMT message instantly caused the state to change to the new value.

@param newState The state that we expect the node to change to.
@param request If true (default), send a CAN message requesting state info.
               If false no request is sent.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *Node::WaitStateChange( NodeState newState, bool request )
{
   CanFrame frame;
   frame.id = nodeID + 0x700;
   frame.type = CAN_FRAME_REMOTE;
   frame.length = 1;

   EventAny e( GUARD_EVENT_DSRDSTATE );

   guardEvents.clrBits( GUARD_EVENT_DSRDSTATE );
   desired = newState;

   if( request ) co->Xmit( frame );

   int32 remain = sdo.GetTimeout();
   int32 to = 20;
   while( remain > 0 )
   {
      const Error *err = e.Wait( guardEvents, to );
      if( err != &ThreadError::Timeout )
	 return err;

      co->Xmit( frame );
      remain -= to;
   }
   return &ThreadError::Timeout;
}

/***************************************************************************/
/**
This method is called when a new node guarding / heartbeat message is received
from the node.
*/
/***************************************************************************/
int Node::NewFrame( CanFrame &frame )
{
   if( frame.type != CAN_FRAME_DATA )
      return 1;

   mutex.Lock();

   // Save the old node state
   NodeState oldState = state;

   // Check for a valid toggle bit.  The toggle bit is only
   // used when running in node guarding mode. 
   int toggleErr = guardToggle ^ (frame.data[0] & 0x80);

   // If the guardToggle variable is -1, then ignore toggle
   // errors.  This is used in the modes where the toggle bit
   // is not defined (heartbeat and no guarding).  It's also
   // used for the first node guarding message since I can't be
   // sure what the node's state is when I start and therefore
   // don't know if the toggle bit should be set.
   if( guardToggle < 0 )
      toggleErr = 0;

   if( !toggleErr )
   {
      switch( frame.data[0] & 0x7f )
      {
	 case 0:   state = NODESTATE_PRE_OP;      break;
	 case 4:   state = NODESTATE_STOPPED;     break;
	 case 5:   state = NODESTATE_OPERATIONAL; break;
	 case 127: state = NODESTATE_PRE_OP;      break;
	 default:  state = NODESTATE_UNKNOWN;     break;
      }

      uint32 bits = GUARD_EVENT_MSG_RVCD;

      // If a thread is waiting for this state, wake it.
      if( state == desired )
      {
	 bits |= GUARD_EVENT_DSRDSTATE;
	 desired = NODESTATE_INVALID;
      }

      // Notify the guarding task that a message was received.
      guardEvents.setBits( bits );
   }
   else
      cml.Warn( "Node %d guard toggle error\n", GetNodeID() );

   // If I'm using the node guarding protocol, update my
   // expected toggle bit value
   if( guardType == GUARDTYPE_NODEGUARD )
      guardToggle = (~frame.data[0]) & 0x80;

   mutex.Unlock();

   // Signal a state change
   if( state != oldState )
      HandleStateChange( oldState, state );

   return 1;
}

/***************************************************************************/
/**
Node object watchdog thread.  This thread monitors the node guarding and 
heartbeat protocols.  If the node stops responding within the required
timeouts, the thread will change the local state variable to indicate the
error.
*/
/***************************************************************************/
void Node::run( void )
{
   CanFrame frame;

   // Initialize my node guarding frame
   frame.id = nodeID + 0x700;
   frame.type = CAN_FRAME_REMOTE;
   frame.length = 1;

   while( 1 )
   {
      // Grab a local copy of the guard parameters safely
      int32 timeout;
      GuardProtocol gtype;

      mutex.Lock();
      timeout = guardTimeout;
      gtype = guardType;
      mutex.Unlock();

      // Clear the message received event bit
      guardEvents.clrBits( GUARD_EVENT_MSG_RVCD );

      /**************************************************
       * For the node guarding protocol; send a remote 
       * transmit request and sleep for the timeout 
       * period.  After sleeping, I see if the message
       * was received.
       **************************************************/
      if( gtype == GUARDTYPE_NODEGUARD )
      {
	 co->Xmit( frame );

	 EventAny e( GUARD_EVENT_CHANGE );

	 if( e.Wait( guardEvents, timeout ) == 0 )
	 {
	    cml.Debug( "Guard %d, event changed\n", nodeID );
	    guardEvents.clrBits( GUARD_EVENT_CHANGE );
	    continue;
	 }

	 // Now, use a zero timeout when getting the
	 // guard semaphore
	 timeout = 0;
      }

      /**************************************************
       * For the heartbeat protocol, just wait for a new
       * message.  This also works when all guarding is
       * disabled since the timeout should be -1 (forever)
       **************************************************/
      EventAny e( GUARD_EVENT_CHANGE | GUARD_EVENT_MSG_RVCD );
      const Error *err = e.Wait( guardEvents, timeout );

      // Handle timeouts
      if( err == &ThreadError::Timeout )
      {
	 cml.Debug( "Guard %d, timeout waiting on response\n", nodeID );
	 mutex.Lock();
	 NodeState oldState = state;
	 state = NODESTATE_GUARDERR;
	 mutex.Unlock();

	 HandleStateChange( oldState, NODESTATE_GUARDERR );
      }

      else if( err )
	 cml.Error( "Guard %d error: %s\n", nodeID, err->toString() );

      else if( e.getMask() & GUARD_EVENT_CHANGE )
      {
	 cml.Debug( "Guard %d, event change\n", nodeID );
	 guardEvents.clrBits( GUARD_EVENT_CHANGE );
      }
   }
}

