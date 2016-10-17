/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This header file defines the classes that define a generic
node on the CANopen network.

*/

#ifndef _DEF_INC_CO_NODE
#define _DEF_INC_CO_NODE

#include "CML_Settings.h"
#include "CML_CanOpen.h"
#include "CML_EventMap.h"
#include "CML_SDO.h"
#include "CML_PDO.h"

CML_NAMESPACE_START()

/**
This class represents node errors. 
There is one static member for each defined node error.
*/
class NodeError: public CanOpenError
{
public:
   /// A node guarding or heartbeat timeout occurred
   static const NodeError GuardTimeout;

protected:
   /// Standard protected constructor
   NodeError( uint16 id, const char *desc ): CanOpenError( id, desc ){}
};

/***************************************************************************/
/**
This class is used by the Node class.  It acts as a receiver
for emergency objects.
*/
/***************************************************************************/
class NodeEmcyRcvr: public Receiver
{
   /// The node that this emergency receiver is associated with
   class Node *node;

   /// Default constructor.  Mark the object as uninitialized.
   NodeEmcyRcvr(){ node = 0; }
   const Error *Init( Node *nodePtr );
   virtual int NewFrame( CanFrame &frame );

   friend class Node;
};

/***************************************************************************/
/**
Enumeration used to identify a CANopen node state.
*/
/***************************************************************************/
enum NodeState
{
   /// Invalid node state
   NODESTATE_INVALID,

   /// Unknown state - the default state on node creation.
   /// The state will be changed when communication with
   /// the node is established.
   NODESTATE_UNKNOWN,

   /// On a node guarding or heartbeat timeout, the state
   /// will change to guard error.
   NODESTATE_GUARDERR,

   /// Stopped state
   NODESTATE_STOPPED,

   /// Pre-operational state
   NODESTATE_PRE_OP,

   /// Operational state
   NODESTATE_OPERATIONAL
};

/***************************************************************************/
/**
Enumeration used to identify the various types of node
guarding protocols used on the CANopen network.       
*/
/***************************************************************************/
enum GuardProtocol 
{
   /// No guarding protocol is in use
   GUARDTYPE_NONE,

   /// The heartbeat protocol is being used
   GUARDTYPE_HEARTBEAT,

   /// Node guarding protocol is being used
   GUARDTYPE_NODEGUARD
};

/***************************************************************************/
/**
CANopen identity object.  Each node is required to include an identity 
object on it's object dictionary at location 0x1018.  The only required
parameter is the vendorID.  All others are included at the manufacturer's 
discretion.
*/
/***************************************************************************/
struct NodeIdentity
{
   /// A unique vendor ID assigned by CiA (Can in Automation)
   uint32 vendorID;
   /// Manufacturer's product code
   uint32 productCode;
   /// Revision number which identifies CANopen functionality
   uint32 revision;
   /// Product serial number
   uint32 serial;
};

/***************************************************************************/
/**
CANopen Node class.  Objects of this class represent individual nodes on 
the CANopen network.
*/
/***************************************************************************/
class Node: public Thread, public Receiver
{
   /// Private copy constructor (not supported)
   Node( const Node& );

   /// Private assignment operator (not supported)
   Node& operator=( const Node& );

public:
   /// This SDO may be used to get/set values in the node's
   /// object dictionary.
   SDO sdo;

   Node();
   Node( CanOpen &co, int16 nodeID );
   virtual ~Node();

   virtual const Error *StopGuarding( void );
   virtual const Error *StartHeartbeat( uint16 period, uint16 timeout );
   virtual const Error *StartNodeGuard( uint16 guardTime, byte lifeFactor );

   virtual const Error *Init( CanOpen &co, int16 nodeID );
   virtual const Error *UnInit( void );

   virtual const Error *PdoSet( uint16 n, PDO &pdo, bool enable=true );

   /// Enable the passed PDO object
   /// @param n The slot number of the PDO
   /// @param pdo The PDO mapped to that slot
   /// @return An error object
   virtual const Error *PdoEnable( uint16 n, PDO &pdo ){
      return PdoEnable( n, pdo.BaseIndex() );
   }

   /// Disable the passed PDO object
   /// @param n The slot number of the PDO
   /// @param pdo The PDO mapped to that slot
   /// @return An error object
   virtual const Error *PdoDisable( uint16 n, PDO &pdo ){
      return PdoDisable( n, pdo.BaseIndex() );
   }

   /// Enable the specified receive PDO.
   /// @param n The slot number of the PDO
   /// @return An error object
   virtual const Error *RpdoEnable( uint16 n ){ return PdoEnable( n, 0x1400 ); }

   /// Disable the specified receive PDO.
   /// @param n The slot number of the PDO
   /// @return An error object
   virtual const Error *RpdoDisable( uint16 n ){ return PdoDisable( n, 0x1400 ); }

   /// Enable the specified transmit PDO.
   /// @param n The slot number of the PDO
   /// @return An error object
   virtual const Error *TpdoEnable( uint16 n ){ return PdoEnable( n, 0x1800 ); }

   /// Disable the specified transmit PDO.
   /// @param n The slot number of the PDO
   /// @return An error object
   virtual const Error *TpdoDisable( uint16 n ){ return PdoDisable( n, 0x1800 ); }

   /// Start this node.
   /// @return An error object
   virtual const Error *StartNode( void )
   { 
      const Error *err = co->StartNode( nodeID );
      if( !err ) err = WaitStateChange( NODESTATE_OPERATIONAL );
      return err;
   }

   /// Stop this node
   /// @return An error object
   virtual const Error *StopNode( void )
   {
      const Error *err = co->StopNode( nodeID );
      if( !err ) err = WaitStateChange( NODESTATE_STOPPED );
      return err;
   }

   /// Put this node in pre-operational state
   /// @return An error object
   virtual const Error *PreOpNode( void )
   {
      const Error *err = co->PreOpNode( nodeID );
      if( !err ) err = WaitStateChange( NODESTATE_PRE_OP );
      return err;
   }

   virtual const Error *ResetNode( void );

   /// Reset this node's communications.
   /// @return An error object
   virtual const Error *ResetComm( void )
   {
      const Error *err = co->ResetComm( nodeID );
      if( !err ) err = WaitStateChange( NODESTATE_PRE_OP );
      return err;
   }

   /// Returns the present state of this node.
   /// Note that this requires node guarding or heartbeats to
   /// be enabled.
   /// @return The present node state.
   virtual NodeState GetState( void ){ return state; }

   /// Read the device type from the object dictionary
   /// @param devType Where the device type is returned
   /// @return An error object
   virtual const Error *GetDeviceType( uint32 &devType ){
      return sdo.Upld32( 0x1000, 0, devType );
   }

   /// Read the error register from the object dictionary
   /// @param err Reference to where the error should be returned.
   /// @return An error object
   virtual const Error *GetErrorRegister( byte &err ){
      return sdo.Upld8( 0x1001, 0, err );
   }

   /// Read the manufacturer status register from the object dictionary.
   /// @param stat Reference to the int32 where the status will be returned
   /// @return An error object
   virtual const Error *GetMfgStatus( uint32 &stat ){
      return sdo.Upld32( 0x1002, 0, stat );
   }

   virtual const Error *GetErrorHistory( uint16 &ct, uint32 *array );

   /// Clear the error history (object 0x1003) array for this node.
   /// @return An error object.
   virtual const Error *ClearErrorHistory( void ){
      return sdo.Dnld32( 0x1003, 0, (uint32)0 );
   }

   /// Read the manufacturer's device name string from the object dictionary
   /// @param len Holds the size of the buffer on entry, and the
   ///            length of the downloaded data on return.
   /// @param str An array of characters used to upload the string.
   /// @return An error object
   virtual const Error *GetMfgDeviceName( int32 &len, char *str ){
      return sdo.UpldString( 0x1008, 0, len, str );
   }

   /// Read the manufacturer's Hardware version string from the object dictionary
   /// @param len Holds the size of the buffer on entry, and the
   ///            length of the downloaded data on return.
   /// @param str An array of characters used to upload the string.
   /// @return An error object
   virtual const Error *GetMfgHardwareVer( int32 &len, char *str ){
      return sdo.UpldString( 0x1009, 0, len, str );
   }

   /// Read the manufacturer's software version string from the object dictionary
   /// @param len Holds the size of the buffer on entry, and the
   ///            length of the downloaded data on return.
   /// @param str An array of characters used to upload the string.
   /// @return An error object
   virtual const Error *GetMfgSoftwareVer( int32 &len, char *str ){
      return sdo.UpldString( 0x100A, 0, len, str );
   }

   virtual const Error *GetIdentity( NodeIdentity &id );

   /// Set the COB-ID of the synch message.  If bit 30 of the
   /// ID is set, then this node will be the synch producer.
   /// @param id COB-ID to set
   /// @return An error object
   virtual const Error *SetSynchId( uint32 id ){
      return sdo.Dnld32( 0x1005, 0, id );
   }

   /// Return the COB-ID of the synch message.  Note that if
   /// this node is producing the synch message, bit 30 will be set.
   /// @param id Where the COB-ID is returned
   /// @return An error object.
   virtual const Error *GetSynchId( uint32 &id ){
      return sdo.Upld32( 0x1005, 0, id );
   }

   /// Set the SYNC message interval in microseconds.
   /// @param per The period in microseconds.
   /// @return An error object.
   virtual const Error *SetSynchPeriod( uint32 per ){
      return sdo.Dnld32( 0x1006, 0, per );
   }

   /// Get the SYNC message interval in microseconds.
   /// @param per Period will be returned here
   /// @return An error object.
   virtual const Error *GetSynchPeriod( uint32 &per ){
      return sdo.Upld32( 0x1006, 0, per );
   }

   /// Start producing SYNC messages on this node.
   /// @return An error object.
   virtual const Error *SynchStart( void ){
      uint32 id;
      const Error *err = GetSynchId( id );
      if( !err )
	 err = SetSynchId( id | 0x40000000 );

      if( !err )
	 co->SetSynchProducer( nodeID );

      return err;
   }

   /// Stop producing SYNC messages on this node.
   /// @return An error object.
   virtual const Error *SynchStop( void ){
      uint32 id;
      const Error *err = GetSynchId( id );
      if( !err && (id&0x40000000) )
	 err = SetSynchId( id & 0x3FFFFFFF );

      if( !err && (co->GetSynchProducer() == nodeID) )
	 co->SetSynchProducer(0);

      return err;
   }

   /// Return the default COB ID value for the receive PDO
   /// with the specified index.  Return -1 if the specified
   /// PDO does not have a default COB ID.
   /// @param index The PDO index
   /// @return The default COB ID for the PDO, or -1 if none.
   virtual int32 GetRpdoCobID( uint16 index )
   {
      if( index > 3 ) return -1;
      return 0x200 + index*0x100 + nodeID;
   }

   /// Return the default COB ID value for the transmit PDO
   /// with the specified index.  Return -1 if the specified
   /// PDO does not have a default COB ID.
   /// @param index The PDO index
   /// @return The default COB ID for the PDO, or -1 if none.
   virtual int32 GetTpdoCobID( uint16 index )
   {
      if( index > 3 ) return -1;
      return 0x180 + index*0x100 + nodeID;
   }

   /// Return the node ID associated with this node
   /// @return The node ID
   virtual int16 GetNodeID( void ){
      return nodeID;
   }

   virtual int NewFrame( CanFrame &frame );
   virtual void run( void );

protected:
   /// Overload this function to handle emergency objects sent
   /// by this node.
   virtual void HandleEmergency( CanFrame &frame ){}

   /// Overload this function to handle changes to the nodes
   /// state.  Note that the state member variable will have
   /// been changed to the new state before this function is
   /// called.
   virtual void HandleStateChange( NodeState from, NodeState to ){}

private:
   const Error *WaitStateChange( NodeState newState, bool request=true );

   const Error *PdoEnable( uint16 n, uint16 base );
   const Error *PdoDisable( uint16 n, uint16 base );

   /// The NodeID for this node.  Note that node IDs must
   /// be in the range 1 to 127.
   int16 nodeID;

   /// This object handles reception of emergency objects
   /// generated by this node.
   NodeEmcyRcvr emcy;

   /// Holds the current state of this node.  Be careful to
   /// avoid race conditions when modifying this variable!
   NodeState state;

   /// This variable identifies the type of guard protocol
   /// being used by this node.  The options are none, heartbeat
   /// monitor, and node guarding.
   GuardProtocol guardType;

   /// This variable gives the guard time in milliseconds.
   /// It's used both in heartbeat and node guarding modes.
   int32 guardTimeout;

   /// This value keeps track of the toggle bit used with 
   /// node guarding.  It's set to -1 if the toggle bit isn't
   /// used.
   int16 guardToggle;

   /// General purpose mutex for this node.
   Mutex mutex;

   /// Used to implement node guarding
   EventMap guardEvents;

   /// State I'm waiting for
   NodeState desired;

   friend class NodeEmcyRcvr;
};

CML_NAMESPACE_END()

#endif

