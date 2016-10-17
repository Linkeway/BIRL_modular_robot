/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
  I/O module object support.  This file holds the code used to implement
  a standard DS401 I/O module.
  */

#include "CML.h"

CML_NAMESPACE_USE();

// I/O module specific error objects
CML_NEW_ERROR( IOError, BadID,      "The passed digital I/O pin ID number is invalid" );
CML_NEW_ERROR( IOError, BadIOCount, "The number of passed I/O ID blocks is invalid" );

/***************************************************************************/
/**
  Default constructor for an I/O module.  
  Any object created using this constructor must be initialized by a call 
  to IOModule::Init before it is used.
  */
/***************************************************************************/
IOModule::IOModule( void )
{
}

/***************************************************************************/
/**
  Construct an IOModule object and initialize it using default settings.
  @param co The CANopen network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  */
/***************************************************************************/
IOModule::IOModule( CanOpen &co, int16 nodeID )
{
   Init( co, nodeID );
}

/***************************************************************************/
/**
  Construct an IOModule object and initialize it using custom settings.
  @param co The CANopen network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @param settings The settings to use when configuring the module
  */
/***************************************************************************/
IOModule::IOModule( CanOpen &co, int16 nodeID, IOModuleSettings &settings )
{
   Init( co, nodeID, settings );
}

/***************************************************************************/
/**
  Virtual destructor for the IOModule object.
  */
/***************************************************************************/
IOModule::~IOModule()
{
}

/***************************************************************************/
/**
  Initialize an I/O module using default settings.  This function associates the 
  object with the CANopen network it will be used on.

  @param co The CANopen network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Init( CanOpen &co, int16 nodeID )
{
   IOModuleSettings settings;
   return Init( co, nodeID, settings );
}

/***************************************************************************/
/**
  Initialize an I/O module using custom settings.  This function associates the 
  object with the CANopen network it will be used on.

  @param co The CANopen network object that this module is associated with.
  @param nodeID The node ID of the module on the network.
  @param settings The settings to use when configuring the module
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Init( CanOpen &co, int16 nodeID, IOModuleSettings &settings )
{
   // Init the base class
   const Error *err = Node::Init( co, nodeID );
   if( err ) return err;

   // Make sure we're in pre-op state.  This allows us to map
   // the PDO objects.
   err = PreOpNode();
   if( err ) return err;

   // Check to see if analog & digital interrupts are enabled.
   // On error, just assume they are
   if( DinGetIntEna( dinIntEna ) ) dinIntEna = true;
   if( AinGetIntEna( ainIntEna ) ) ainIntEna = true;

#ifdef CML_ENABLE_IOMODULE_PDOS
   // Check the number of 8-bit digital output blocks.  If this is
   // more then zero then I'll map them (up to 8) to the first 
   // receive PDO.  Note that I don't check errors here.  If this
   // fails the count will come back as zero.
   uint8 ct;
   Dout8GetCt( ct );

   if( ct && settings.useStandardDoutPDO )
   {
      if( ct > 8 ) ct = 8;

      uint8 ids[8];
      for( int i=0; i<ct; i++ ) ids[i] = i;

      err = doutPDO.Init( this, GetRpdoCobID(0), ct, ids );

      if( !err ) err = PdoSet( 0, doutPDO );
      if( err ) return err;
   }

   // Map up to 12 16-bit analog outputs to PDOs.  These are the
   // standard analog output PDOs defined by the spec.
   if( settings.useStandardAoutPDO )
   {
      Aout16GetCt( ct );

      for( int n=0; n<3 && ct>0; n++ )
      {
	 uint8 map = ct;
	 if( map > 4 ) map = 4;
	 ct -= map;

	 uint8 ids[4];
	 for( int i=0; i<map; i++ ) ids[i] = 4*n+i;

	 err = aoutPDO[n].Init( this, GetRpdoCobID(n+1), map, ids );

	 if( !err ) err = PdoSet( n+1, aoutPDO[n] );
	 if( err ) return err;
      }
   }

   // Check the number of 8-bit digital input blocks.  If this is
   // more then zero then I'll map them (up to 8) to the first 
   // transmit PDO.  Note that I don't check errors here.  If this
   // fails the count will come back as zero.
   Din8GetCt( ct );
   if( ct && settings.useStandardDinPDO )
   {
      if( ct > 8 ) ct = 8;

      uint8 ids[8];
      for( int i=0; i<ct; i++ ) ids[i] = i;

      err = dinPDO.Init( this, GetTpdoCobID(0), ct, ids, IOEVENT_DIN_PDO0 );

      if( !err ) err = PdoSet( 0, dinPDO );
      if( err ) return err;
   }

   // Map up to 12 16-bit analog inputs to PDOs.  These are the
   // standard analog input PDOs defined by the spec.
   if( settings.useStandardAinPDO )
   {
      Ain16GetCt( ct );

      for( int n=0; n<3 && ct>0; n++ )
      {
	 uint8 map = ct;
	 if( map > 4 ) map = 4;
	 ct -= map;

	 uint8 ids[4];
	 for( int i=0; i<map; i++ ) ids[i] = 4*n+i;

	 err = ainPDO[n].Init( this, GetTpdoCobID(n+1), map, ids, 
	                       (IOMODULE_EVENTS)(IOEVENT_AIN_PDO0<<n) );

	 if( !err ) err = PdoSet( n+1, ainPDO[n] );
	 if( err ) return err;
      }
   }
#endif

   // Setup heartbeat or node guarding
   if( settings.heartbeatPeriod )
      err = StartHeartbeat( settings.heartbeatPeriod, settings.heartbeatTimeout );

   else if( settings.guardTime && settings.lifeFactor )
      err = StartNodeGuard( settings.guardTime, settings.lifeFactor );

   else
      err = StopGuarding();

   if( err ) return err;

   // Put the node into operational state 
   return StartNode();
}

/***************************************************************************/
/**
  Write an individual digital output.

  The output may be written either by SDO or by PDO.  The PDO method 
  is faster since it only requires a single message to be sent.  SDO 
  transfers additionally require a response from the module.  

  If a PDO transfer is requested, but is not possible because the module is
  not in an operational state, or because the output isn't mapped to
  the PDO, then an SDO transfer will be used.

  @param id Identifies which output to write.
  @param value The new value of the output line.
  @param viaSDO If true, the outputs will be written using SDO messages.
  If false (default), then a PDO will be used if possible.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::DoutWrite( uint16 id, bool value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   // Update the bit value in the PDO to keep my local data current.
   // If this fails, then the bit isn't mapped and I'll have to use
   // an SDO to update it.
   if( !doutPDO.UpdateBit( id, value ) )
      viaSDO = true;

   // Output this using a PDO if requested and possible
   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return doutPDO.Transmit();

   else
#endif
      return BitDnld( IOOBJID_DOUT_1_VALUE, id, value );
}

/***************************************************************************/
/**
  Write a group of 8 digital outputs.

  The outputs may be written either by SDO or by PDO.  The PDO method 
  is faster since it only requires a single message to be sent.  SDO 
  transfers additionally require a response from the module.  

  If a PDO transfer is requested, but is not possible because the module is
  not in an operational state, or because the output block isn't mapped to
  the PDO, then an SDO transfer will be used.

  @param id Identifies which group of outputs to write.   
  @param value The new value of the output lines.         
  @param viaSDO If true, the outputs will be written using SDO messages.
  If false (default), then a PDO will be used if possible.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Dout8Write( uint8 id, uint8 value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   // Update the PDO even if we are using an SDO.  This keeps
   // my PDO data up to date.
   if( !doutPDO.Update( id, value ) )
      viaSDO = true;

   // Output this using a PDO if requested and possible
   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return doutPDO.Transmit();

   else
#endif
      return sdo.Dnld8( IOOBJID_DOUT_8_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Write a group of 16 digital outputs.

  The outputs may be written either by SDO or by PDO.  The PDO method 
  is faster since it only requires a single message to be sent.  SDO 
  transfers additionally require a response from the module.  

  If a PDO transfer is requested, but is not possible because the module is
  not in an operational state, or because the output block isn't mapped to
  the PDO, then an SDO transfer will be used.

  @param id Identifies which group of outputs to write.
  @param value The new value of the output lines.
  @param viaSDO If true, the outputs will be written using SDO messages.
  If false (default), then a PDO will be used if possible.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Dout16Write( uint8 id, uint16 value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   if( id >= 127 ) 
      viaSDO = true;

   else if( !doutPDO.Update( 2*id, (uint8)value ) )
      viaSDO = true;

   else if( !doutPDO.Update( 2*id+1, (uint8)(value>>8) ) )
      viaSDO = true;

   // Output this using a PDO if requested and possible
   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return doutPDO.Transmit();

   else
#endif
      return sdo.Dnld16( IOOBJID_DOUT_16_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Write a group of 32 digital outputs.

  The outputs may be written either by SDO or by PDO.  The PDO method 
  is faster since it only requires a single message to be sent.  SDO 
  transfers additionally require a response from the module.  

  If a PDO transfer is requested, but is not possible because the module is
  not in an operational state, or because the output block isn't mapped to
  the PDO, then an SDO transfer will be used.

  @param id Identifies which group of outputs to write.
  @param value The new value of the output lines.
  @param viaSDO If true, the outputs will be written using SDO messages.
  If false (default), then a PDO will be used if possible.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Dout32Write( uint8 id, uint32 value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   if( id >= 63 )
      viaSDO = true;

   else if( !doutPDO.Update( 4*id, (uint8)value ) )
      viaSDO = true;

   else if( !doutPDO.Update( 4*id+1, (uint8)(value>>8) ) )
      viaSDO = true;

   else if( !doutPDO.Update( 4*id+2, (uint8)(value>>16) ) )
      viaSDO = true;

   else if( !doutPDO.Update( 4*id+3, (uint8)(value>>24) ) )
      viaSDO = true;

   // Output this using a PDO if requested and possible
   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return doutPDO.Transmit();

   else
#endif
      return sdo.Dnld32( IOOBJID_DOUT_32_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Write to a 16-bit analog output.  Since 16-bit outputs are mapped to the
  default PDOs of the I/O module, these outputs may be written using either
  PDOs or SDOs.  

  @param id The analog input channel ID
  @param value The value to write.
  @param viaSDO If true, the outputs will be written using SDO messages.
  If false (default), then a PDO will be used if possible.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Aout16Write( uint8 id, int16 value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   // Update the local copies stored by the PDOs regardless of whether
   // we are using the PDO for output.  This keeps me synchronized.
   int pdo = -1;
   for( int i=0; i<3; i++ )
   {
      if( aoutPDO[i].Update( id, value ) )
	 pdo = i;
   }

   // Output this using a PDO if requested and possible
   if( (!viaSDO) && (pdo>=0) && (GetState() == NODESTATE_OPERATIONAL) )
      return aoutPDO[pdo].Transmit();

   else
#endif
      return sdo.Dnld16( IOOBJID_AOUT_16_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Read a single digital input.

  @param id Identifies the digital input to read.
  @param value The value of the input.
  @param viaSDO If true, an SDO will be used to read the input pin.  If false
  (default), the latest value returned via PDO will be returned, if
  available.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::DinRead( uint16 id, bool &value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   if( !dinIntEna || !dinPDO.GetBitVal( id, value ) )
      viaSDO = true;

   if( (!viaSDO) && (GetState()==NODESTATE_OPERATIONAL) )
      return 0;

   else
#endif
      return BitUpld(IOOBJID_DIN_1_VALUE, id, value );
}

/***************************************************************************/
/**
  Read a group of 8 digital inputs.  
  @param id Identifies which group of 8 inputs to read.  
  @param value The value of the 8 input lines is returned here.
  @param viaSDO If true, read the inputs using SDO transfers.  If false
  (default) use the most recently received PDO data if this input
  group is mapped to a transmit PDO and the PDO is active.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Din8Read( uint8 id, uint8 &value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   if( !dinIntEna || !dinPDO.GetInVal( id, value ) )
      viaSDO = true;

   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return 0;

   else
#endif
      return sdo.Upld8( IOOBJID_DIN_8_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Read a group of 16 digital inputs.  
  @param id Identifies which group of 16 inputs to read.  
  @param value The value of the 16 input lines is returned here.
  @param viaSDO If true, read the inputs using SDO transfers.  If false
  (default) use the most recently received PDO data if this input
  group is mapped to a transmit PDO and the PDO is active.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Din16Read( uint8 id, uint16 &value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   uint8 v[2];

   if( id >= 127 ) viaSDO = true;
   else if( !dinIntEna ) viaSDO = true;
   else if( !dinPDO.GetInVal( 2*id,   v[0] ) ) viaSDO = true;
   else if( !dinPDO.GetInVal( 2*id+1, v[1] ) ) viaSDO = true;

   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
   {
      value = bytes_to_uint16( v );
      return 0;
   }

   else
#endif
      return sdo.Upld16( IOOBJID_DIN_16_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Read a group of 32 digital inputs.  
  @param id Identifies which group of 32 inputs to read.  
  @param value The value of the 32 input lines is returned here.
  @param viaSDO If true, read the inputs using SDO transfers.  If false
  (default) use the most recently received PDO data if this input
  group is mapped to a transmit PDO and the PDO is active.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Din32Read( uint8 id, uint32 &value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   uint8 v[4];

   if( id >= 63 ) viaSDO = true;
   else if( !dinIntEna ) viaSDO = true;
   else if( !dinPDO.GetInVal( 4*id,   v[0] ) ) viaSDO = true;
   else if( !dinPDO.GetInVal( 4*id+1, v[1] ) ) viaSDO = true;
   else if( !dinPDO.GetInVal( 4*id+2, v[2] ) ) viaSDO = true;
   else if( !dinPDO.GetInVal( 4*id+3, v[3] ) ) viaSDO = true;

   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
   {
      value = bytes_to_uint32( v );
      return 0;
   }

   else
#endif
      return sdo.Upld32( IOOBJID_DIN_32_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Read a 16-bit analog input.
  @param id The analog input channel ID
  @param value The analog input value
  @param viaSDO If true, read the input using SDO transfers.  If false
  (default) use the most recently received PDO data if this input
  is mapped to a transmit PDO and the PDO is active.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::Ain16Read( uint8 id, int16 &value, bool viaSDO )
{
#ifdef CML_ENABLE_IOMODULE_PDOS
   if( !ainIntEna )
      viaSDO = true;

   else
   {
      bool ok = false;

      for( int i=0; i<3 && !ok; i++ )
	 ok = ainPDO[i].GetInVal( id, value );

      if( !ok ) viaSDO = true;
   }

   if( (!viaSDO) && (GetState() == NODESTATE_OPERATIONAL) )
      return 0;

   else
#endif
      return sdo.Upld16( IOOBJID_AIN_16_VALUE, id+1, value );
}

/***************************************************************************/
/**
  Wait on an event associated with this I/O module.  The standard events are 
  used to indicate that a new transmit PDO has been received.  A thread may 
  wait on such an event by calling this function.

  @param event The event(s) to wait on.  Multiple events may be ORed together
  and in this case this function will return when any of them occur.
  @param timeout The timeout for the wait (milliseconds).  Negative values 
  indicate that no timeout should be used (wait forever).  The default
  value is -1.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::WaitIOEvent( IOMODULE_EVENTS event, int32 timeout )
{
   EventAny any( (uint32)event );
   return any.Wait( eventMap, timeout );
}

/***************************************************************************/
/**
  Wait for an event associated with this I/O module.  This function can be used
  to wait on any generic event associated with the I/O module.
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match Returns the matching event condition.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *IOModule::WaitIOEvent( Event &e, int32 timeout, IOMODULE_EVENTS &match )
{
   return e.Wait( eventMap, timeout );
}

#ifdef CML_ENABLE_IOMODULE_PDOS
/***************************************************************************/
/**
  Initialize a digital output PDO object.
  @param io Pointer to the I/O module to which this PDO is assigned.
  @param cobID The CAN ID for this PDO message.
  @param ct The number of output blocks to be mapped (1 to 8)
  @param id An array of ct output block ID numbers.  These will be mapped
  (in order) to the PDO.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::DigOutPDO::Init( IOModule *io, uint32 cobID, uint8 ct, uint8 id[] )
{
   if( ct < 1 || ct > 8 )
      return &IOError::BadIOCount;

   this->io = io;

   const Error *err = RPDO::Init( cobID );

   if( !err ) err = SetType( 255 );

   for( uint8 i=0; i<ct; i++ )
   {
      if( !err ) err = out[i].Init( IOOBJID_DOUT_8_VALUE, id[i]+1 );
      if( !err ) err = AddVar( out[i] );
   }

   return err;
}

/***************************************************************************/
/**
  Update the locally stored value of one of the 8-bit digital output blocks 
  associated with this PDO.
  @param id The output block ID to be updatad.
  @param value The new value for the output block.
  @return true if the value was updated, 
  false if the block isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::DigOutPDO::Update( uint8 id, uint8 value  )
{
   ++id;

   for( int i=0; i<mapCt; i++ )
   {
      if( out[i].GetSub() == id )
      {
	 out[i].Write( value );
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  Update the locally stored value of one bit in this PDO.
  @param id The output ID to be updatad.
  @param value The new value for the output.
  @return true if the value was updated, 
  false if the output isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::DigOutPDO::UpdateBit( uint16 id, bool value )
{
   uint8 block = 1 + (id>>3);
   uint8 mask  = 1 << (id&3);

   for( int i=0; i<mapCt; i++ )
   {
      if( out[i].GetSub() == block )
      {
	 uint8 v = out[i].Read();

	 if( value ) v |=  mask;
	 else        v &= ~mask;

	 out[i].Write( v );
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  Transmit this PDO.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::DigOutPDO::Transmit( void )
{
   return RPDO::Transmit( io->GetCanOpen() );
}

/***************************************************************************/
/**
  Initialize an analog output PDO object.
  @param io Pointer to the I/O module to which this PDO is assigned.
  @param cobID The CAN ID for this PDO message.
  @param ct The number of outputs to be mapped (1 to 4)
  @param id An array of ct output ID numbers.  These will be mapped
  (in order) to the PDO.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::AlgOutPDO::Init( class IOModule *io, uint32 cobID, uint8 ct, uint8 id[] )
{
   if( ct < 1 || ct > 4 )
      return &IOError::BadIOCount;

   this->io = io;

   const Error *err = RPDO::Init( cobID );

   if( !err ) err = SetType( 255 );

   for( uint8 i=0; i<ct; i++ )
   {
      if( !err ) err = out[i].Init( IOOBJID_AOUT_16_VALUE, id[i]+1 );
      if( !err ) err = AddVar( out[i] );
   }

   return err;
}

/***************************************************************************/
/**
  Update the locally stored value of one of the 16-bit analog outputs
  associated with this PDO.

  @param id The output block ID to be updatad.
  @param value The new value for the output block.
  @return true if the value was updated, 
  false if the block isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::AlgOutPDO::Update( uint8 id, int16 value )
{
   ++id;

   for( int i=0; i<mapCt; i++ )
   {
      if( out[i].GetSub() == id )
      {
	 out[i].Write( value );
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  Transmit this PDO.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::AlgOutPDO::Transmit( void )
{
   return RPDO::Transmit( io->GetCanOpen() );
}

/***************************************************************************/
/**
  Initialize a digital input PDO object.
  @param io Pointer to the I/O module to which this PDO is assigned.
  @param cobID The CAN ID for this PDO message.
  @param ct The number of input blocks to be mapped (1 to 8)
  @param id An array of ct input block ID numbers.  These will be mapped
  (in order) to the PDO.
  @param event The event bit to post when a PDO message is received.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::DigInPDO::Init( class IOModule *io, uint32 cobID, uint8 ct, 
                                       uint8 id[], IOMODULE_EVENTS event )
{
   if( ct < 1 || ct > 8 )
      return &IOError::BadIOCount;

   this->io = io;
   eventMask = event;

   const Error *err = TPDO::Init( io->GetCanOpen(), cobID );

   if( !err ) err = SetType( 255 );

   for( uint8 i=0; i<ct; i++ )
   {
      if( !err ) err = in[i].Init( IOOBJID_DIN_8_VALUE, id[i]+1 );
      if( !err ) err = AddVar( in[i] );

      // Read the initial value of this bank of inputs
      uint8 value;
      if( !err ) err = io->sdo.Upld8( IOOBJID_DIN_8_VALUE, id[i]+1, value );
      in[i].Write( value );
   }

   if( !err ) err = EnableReceiver();

   return err;
}

/***************************************************************************/
/**
  Read the specified input bank from the PDO's cached data.  The value returned
  will be the last value received via PDO for this input bank.
  @param id The input block ID to be checked.
  @param value The input value for the block will be returned here.
  @return true if the value was returned,
  false if the block isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::DigInPDO::GetInVal( uint8 id, uint8 &value )
{
   ++id;

   for( int i=0; i<mapCt; i++ )
   {
      if( in[i].GetSub() == id )
      {
	 value = in[i].Read();
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  Update the locally stored value of one bit in this PDO.
  @param id The output ID to be updatad.
  @param value The new value for the output.
  @return true if the value was updated, 
  false if the output isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::DigInPDO::GetBitVal( uint16 id, bool &value )
{
   uint8 block = 1 + (id>>3);
   uint8 mask  = 1 << (id&3);

   for( int i=0; i<mapCt; i++ )
   {
      if( in[i].GetSub() == block )
      {
	 value = (in[i].Read() & mask) == mask;
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  New transmit PDO received.  This method is called by the CANopen reader thread
  when a new PDO message is received.  It causes this PDO object to post it's 
  event to the IOModule object's event map.  This will cause any waiting threads
  to wake up.
  */
/***************************************************************************/
void IOModule::DigInPDO::Received( void )
{
   io->PostIOEvent( eventMask );
}

/***************************************************************************/
/**
  Initialize a analog input PDO object.
  @param io Pointer to the I/O module to which this PDO is assigned.
  @param cobID The CAN ID for this PDO message.
  @param ct The number of inputs to be mapped (1 to 4)
  @param id An array of ct input ID numbers.  These will be mapped
  (in order) to the PDO.
  @param event The event bit to post when a PDO message is received.
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *IOModule::AlgInPDO::Init( class IOModule *io, uint32 cobID, uint8 ct, 
                                       uint8 id[], IOMODULE_EVENTS event )
{
   if( ct < 1 || ct > 4 )
      return &IOError::BadIOCount;

   this->io = io;
   eventMask = event;

   const Error *err = TPDO::Init( io->GetCanOpen(), cobID );

   if( !err ) err = SetType( 255 );

   for( uint8 i=0; i<ct; i++ )
   {
      if( !err ) err = in[i].Init( IOOBJID_AIN_16_VALUE, id[i]+1 );
      if( !err ) err = AddVar( in[i] );

      // Read the initial value of this input
      int16 value;
      if( !err ) err = io->sdo.Upld16( IOOBJID_AIN_16_VALUE, id[i]+1, value );
      in[i].Write( value );
   }

   if( !err ) err = EnableReceiver();

   return err;
}

/***************************************************************************/
/**
  Read the specified input from the PDO's cached data.  The value returned
  will be the last value received via PDO for this input bank.
  @param id The input ID to be checked.
  @param value The input value will be returned here.  If the input is not 
  mapped to this PDO, then this will not be changed.
  @return true if the value was returned,
  false if the input isn't mapped to this PDO.
  */
/***************************************************************************/
bool IOModule::AlgInPDO::GetInVal( uint8 id, int16 &value )
{
   ++id;

   for( int i=0; i<mapCt; i++ )
   {
      if( in[i].GetSub() == id )
      {
	 value = in[i].Read();
	 return true;
      }
   }

   return false;
}

/***************************************************************************/
/**
  New transmit PDO received.  This method is called by the CANopen reader thread
  when a new PDO message is received.  It causes this PDO object to post it's 
  event to the IOModule object's event map.  This will cause any waiting threads
  to wake up.
  */
/***************************************************************************/
void IOModule::AlgInPDO::Received( void )
{
   io->PostIOEvent( eventMask );
}
#endif
