/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

  This file provides most of the implementation for the Copley Amplifier object.

  Since the Amp object is large and complex, it's member functions have been 
  split into several files:

  - This file: Contains the core code.

  - AmpParam.cpp: Holds functions used to upload and download various
  blocks of amplifier parameters.

  - AmpPDO.cpp: Contains functions used to implement the various PDO objects
  used in conjunction with the Amp object.

*/

#include "CML.h"


CML_NAMESPACE_USE();

// Amplifier specific error objects
CML_NEW_ERROR( AmpError, NodeState,     "Drive state invalid for operation" );
CML_NEW_ERROR( AmpError, pvtSegPos,     "PVT segment position out of range" );
CML_NEW_ERROR( AmpError, pvtSegVel,     "PVT segment velocity out of range" );
CML_NEW_ERROR( AmpError, pvtBufferFull, "PVT trajectory buffer full" );
CML_NEW_ERROR( AmpError, badDeviceID,   "Device does not seem to be a Copley amplifier" );
CML_NEW_ERROR( AmpError, badHomeParam,  "Bad parameter passed to home function" );
CML_NEW_ERROR( AmpError, badMoveParam,  "Bad parameter passed to move function" );
CML_NEW_ERROR( AmpError, InMotion,      "The amplifier is currently executing a move" );
CML_NEW_ERROR( AmpError, GuardError,    "The amplifier did not respond to a node guarding or heartbeat message in time" );
CML_NEW_ERROR( AmpError, Fault,         "The amplifier detected a latching fault" );
CML_NEW_ERROR( AmpError, ShortCircuit,  "The amplifier detected a short circuit condition" );
CML_NEW_ERROR( AmpError, AmpTemp,       "The amplifier detected an over temperature error" );
CML_NEW_ERROR( AmpError, MotorTemp,     "The amplifier detected a motor temperature error" );
CML_NEW_ERROR( AmpError, OverVolt,      "The amplifier detected an over voltage condition" );
CML_NEW_ERROR( AmpError, UnderVolt,     "The amplifier detected an under voltage condition" );
CML_NEW_ERROR( AmpError, EncoderPower,  "The amplifier detected an encoder power error" );
CML_NEW_ERROR( AmpError, PhaseErr,      "The amplifier detected a phasing error" );
CML_NEW_ERROR( AmpError, TrackErr,      "The amplifier detected a tracking error." );
CML_NEW_ERROR( AmpError, PosLim,        "Positive limit switch is active" );
CML_NEW_ERROR( AmpError, NegLim,        "Negative limit switch is active" );
CML_NEW_ERROR( AmpError, PosSoftLim,    "Positive software limit is active" );
CML_NEW_ERROR( AmpError, NegSoftLim,    "Negative software limit is active" );
CML_NEW_ERROR( AmpError, TrackWarn,     "Position tracking warning" );
CML_NEW_ERROR( AmpError, Unknown,       "An unknown amplifier error occurred" );
CML_NEW_ERROR( AmpError, Reset,         "An amplifier reset was detected" );
CML_NEW_ERROR( AmpError, Disabled,      "The amplifier is currently disabled" );
CML_NEW_ERROR( AmpError, QuickStopMode, "The amplifier is currently in quick stop mode" );
CML_NEW_ERROR( AmpError, NoUserUnits,   "User units are not enabled in CML_Settings.h" );
CML_NEW_ERROR( AmpError, Abort,         "Trajectory aborted" );
CML_NEW_ERROR( AmpError, pvtPosUnavail, "The PVT segment position is not available." );
CML_NEW_ERROR( AmpError, VelWin,        "Velocity tracking window exceeded." );

CML_NEW_ERROR( AmpFault, Memory,        "Fatal hardware error: Amplifier flash data is corrupt." );
CML_NEW_ERROR( AmpFault, ADC,           "Fatal hardware error: An A/D offset error has occurred." );
CML_NEW_ERROR( AmpFault, ShortCircuit,  "The amplifier latched a short circuit condition" );
CML_NEW_ERROR( AmpFault, AmpTemp,       "The amplifier latched an over temperature error" );
CML_NEW_ERROR( AmpFault, MotorTemp,     "The amplifier latched a motor temperature error" );
CML_NEW_ERROR( AmpFault, OverVolt,      "The amplifier latched an over voltage condition" );
CML_NEW_ERROR( AmpFault, UnderVolt,     "The amplifier latched an under voltage condition" );
CML_NEW_ERROR( AmpFault, EncoderPower,  "The amplifier latched an encoder power error" );
CML_NEW_ERROR( AmpFault, PhaseErr,      "The amplifier latched a phasing error" );
CML_NEW_ERROR( AmpFault, TrackErr,      "The amplifier latched a tracking error." );
CML_NEW_ERROR( AmpFault, I2TLimit,      "Current limited by i^2t algorithm." );
CML_NEW_ERROR( AmpFault, Unknown, 	"Some unknown amplifier latched fault has occurred" );

CML_NEW_ERROR( TrjError, NoneAvailable, "No trajectory segments currently available" );

// local functions
static bool isCanMode( AMP_MODE mode );

/***************************************************************************/
/**
  Construct and initialize an amplifier object.
  @param co Reference to the CANopen network for this amp.
  @param nodeID a valid node ID for the amp
  @param settings Amplifier settings to be used.
  */
/***************************************************************************/
Amp::Amp( CanOpen &co, int16 nodeID, AmpSettings &settings )
{
   myLink = 0;
   Init( co, nodeID, settings );
}

/***************************************************************************/
/**
  Construct and initialize an amplifier object using defaults for 
  all amp settings.
  @param co Reference to the CANopen network for this amp.
  @param nodeID a valid node ID for the amp
  */
/***************************************************************************/
Amp::Amp( CanOpen &co, int16 nodeID )
{
   myLink = 0;
   Init( co, nodeID );
}

/***************************************************************************/
/**
  Amp object destructor.
  */
/***************************************************************************/
Amp::~Amp()
{
   if( myLink )
   {
      cml.Error( "Amp: %d destroyed while owned by a linkage!\n", GetNodeID() );
      myLink->InvalidateAmp( this );
   }
   else
      cml.Debug( "Amp: %d destroyed\n", GetNodeID() );
}

/***************************************************************************/
/**
  Initialize the amplifier object using all default settings.
  @param co Reference to the CANopen network for this amp.
  @param nodeID a valid node ID for the amp
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::Init( CanOpen &co, int16 nodeID )
{
   AmpSettings settings;
   return Init( co, nodeID, settings );
}

/***************************************************************************/
/**
  Initialize the amplifier object with custom amp settings.
  @param co Reference to the CANopen network for this amp.
  @param nodeID a valid node ID for the amp
  @param settings Amplifier settings to be used.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::Init( CanOpen &co, int16 nodeID, AmpSettings &settings )
{
   int i=0;

   cml.Debug( "Amp %d Init\n", nodeID );

   // Initialize the base class
   const Error *err = Node::Init( co, nodeID );
   if( err ) goto retErr;

   if( settings.resetOnInit )
      ResetNode();

   // Make sure this is the right type of node.
   NodeIdentity id;
   cml.Debug( "Amp %d, checking ID\n", nodeID );
   err = GetIdentity( id );
   if( err ) goto retErr;

   if( id.vendorID != 0x000000AB )
   {
      err = &AmpError::badDeviceID;
      goto retErr;
   }

   // Init some local variables
   initialSettings = settings;
   pvtTrj = 0;
   sdo.blkUpldOK = true;

   // If using programmable units, default to units
   // of encoder counts.
#ifdef CML_ENABLE_USER_UNITS
   err = SetCountsPerUnit( 1.0 );
   if( err ) goto retErr;
#endif

   // Read and save the amplifiers software version number 
   err = sdo.Upld16( OBJID_AMP_INFO, 24, SwVersionNum );

   // Disable all the default PDOs 
   cml.Debug( "Amp %d, Initting PDOs\n", nodeID );
   for( i=0; i<8; i++ )
   {
      if( !err ) err = TpdoDisable( i );
      if( !err ) err = RpdoDisable( i );
   }

   // I always start out with the following stopping modes:
   // quick stop - use the quick stop ramp
   // halt - use the profile deceleration
   cml.Debug( "Amp %d, setting default stop modes\n", nodeID );
   if( !err ) err = SetQuickStop( QSTOP_QUICKSTOP );
   if( !err ) err = SetHaltMode( HALT_DECEL );

   // Read the initial mode information from the amplifier
   uint16 desiredState;
   uint8 canOpMode;
   if( !err ) err = sdo.Upld16( OBJID_AMP_MODE, 0, desiredState );
   if( !err ) err = sdo.Upld8( OBJID_OP_MODE, 0, canOpMode );
   if( !err ) err = sdo.Upld16( OBJID_CONTROL, 0, lastCtrlWord );
   if( !err ) err = GetHomeMethod( lastHomeMethod );
   if( err ) goto retErr;

   lastMode = (AMP_MODE)( (desiredState<<8) | canOpMode );
   cml.Debug( "Amp %d, Initial mode is 0x%04x\n", nodeID, lastMode );

   if( !desiredState || !isCanMode( lastMode ) )
      lastMode = (AMP_MODE)( lastMode & 0xFF00 );

   // The default control method is based on the 
   // type of amplifier.
   err = sdo.Upld16( OBJID_AMP_INFO, 13, hwType );
   if( err ) goto retErr;

   // Try to figure out if this is primarily a servo drive or a
   // microstepper.  This sets the default control mode which can 
   // be overriden by specifying it when setting the new mode.
   //
   // For older generation amps, we can tell by the hardware type.
   if( (hwType & 0xFF00) == 0x0200 )
   {
      canCtrlMethod = ((hwType&0xFFC0)==0x0240) ?  AMPMODE_CAN_USTEP : AMPMODE_CAN_SERVO;
   }

   // For newer generation amps, we will read the motor type and
   // select based on whether the amp is set up for a servo or stepper
   // motor.
   else
   {
      uint16 mtrType;
      err = sdo.Upld16( OBJID_MOTOR_INFO,  1, mtrType );
      if( err ) goto retErr;
      canCtrlMethod = ((mtrType&0x00F0)==0x0020) ?  AMPMODE_CAN_USTEP : AMPMODE_CAN_SERVO;
   }

   // Initialize my status PDO
   if( !err ) err = statPdo.Init( *this, 0 );

   // Init my 'PVT buffer status' PDO
   if( !err ) err = buffStatPdo.Init( *this, 1 );

   // Initialize my PVT segment PDO
   if( !err ) err = pvtPdo.Init( *this, 0 );
   if( err ) goto retErr;

   // If the amplifier should be disabled on startup, 
   // clear the control word.  This ensures that the
   // amp won't be enabled when I start it.
   enabled = settings.enableOnInit;
   if( !enabled )
      err = Disable( false );

   // Set the initial requested mode
   if( !err ) err = SetAmpMode( settings.initialMode );

   // Start the node
   cml.Debug( "Amp %d, starting node\n", nodeID );
   if( !err ) err = StartNode();
   if( err ) goto retErr;

   // Request a status PDO update and wait for it to be
   // received before continuing.
   cml.Debug( "Amp %d, (type 0x%04x) Getting initial status\n", nodeID, hwType );
   err = RequestStatUpdt();

   if( !err )
   {
      EventNone e(AMPEVENT_NOT_INIT);
      err = WaitEvent( e, sdo.GetTimeout() );
   }

   // Find the value that the amp expects for the next PVT segment
   // ID.  Normally this will be zero (after amp reset).
   if( !err ) err = GetPvtSegID( pvtSegID );

   // Setup the synch message
   cml.Debug( "Amp %d, Setting up synch\n", nodeID );
   if( !err ) err = SetSynchPeriod( settings.synchPeriod );
   if( !err ) err = SetSynchId( settings.synchID );

   // See if we are picking our own synch producer
   // If so, pick the first initialized amplifier.
   if( !err && settings.synchUseFirstAmp )
   {
      settings.synchProducer = (co.GetSynchProducer() == 0);
   }

   if( err ) goto retErr;

   // Now, start the synch if we are the producer.
   if( settings.synchProducer )
   {
      cml.Debug( "Amp %d, Starting synch production\n", nodeID );
      err = SynchStart();
   }
   else
   {
      cml.Debug( "Amp %d, Stopping synch production\n", nodeID );
      err = SynchStop();
   }

   if( err ) goto retErr;

   // Setup heartbeat or node guarding
   cml.Debug( "Amp %d, setting up node guarding\n", nodeID );
   if( settings.heartbeatPeriod )
      err = StartHeartbeat( settings.heartbeatPeriod, settings.heartbeatTimeout );

   else if( settings.guardTime && settings.lifeFactor )
      err = StartNodeGuard( settings.guardTime, settings.lifeFactor );

   else
      err = StopGuarding();

   if( err ) goto retErr;

   // Setup the PDO used to synchronize amplifiers over the network
   cml.Debug( "Amp %d, setting up time synch PDO\n", nodeID );
   err = SetupSynchPDO( settings );
   if( err ) goto retErr;

   // Clear the latched version of the amplifier's 
   // event status register.  We use this register
   // to check for unexpected amplifier resets.
   cml.Debug( "Amp %d, clearing event latch\n", nodeID );
   err = ClearEventLatch( (EVENT_STATUS)0xFFFFFFFF );
   if( err ) goto retErr;

   // Clear any latched fault conditions
   cml.Debug( "Amp %d, clearing faults\n", nodeID );
   err = ClearFaults();
   if( err ) goto retErr;

   // Now, try to enable the amplifier if requested
   if( settings.enableOnInit )
   {
      AMP_EVENT evnt;

      cml.Debug( "Amp %d, Enabling amp\n", nodeID );

      // Wait for any non-latching errors to clear.
      // This can occur on the Xenus as it's high
      // voltage comes up.  I'll wait 400ms which 
      // is about twice what this should take max.
      EventNone e(AMPEVENT_ERROR);
      WaitEvent( e, 400 );

      // Now, try to enable the amplifier.
      err = Enable();

      // On failure, try to return a useful error
      // message rather then just a timeout.
      if( err )
      {
	 if( err == &ThreadError::Timeout )
	    err = GetErrorStatus( false );
	 goto retErr;
      }

      // Make sure the amplifier is really enabled.
      // The fact that the Enable() function returned success
      // means that the amplifier is software enabled (i.e. 
      // allowed to enable if possible), but it could still be
      // disabled by other factors.
      err = GetEventMask( evnt );
      if( err ) goto retErr;

      if( evnt & AMPEVENT_DISABLED )
      {
	 // Check the lower level 'event status' register
	 // returned with the most recent status PDO.
	 // This should give me more info on what's holding
	 // me up.
	 EVENT_STATUS estat = (EVENT_STATUS)statPdo.estat.Read();

	 // The only good reason for the amplifier to be disabled
	 // at this point is if an input pin is configured to disable
	 // it.  If that's not the case, I'll give the amplifier a
	 // little time to enable on it's own.  This is helpful for the
	 // Xenus amplifier which has a built in delay after closing
	 // it's high voltage relay.
	 if( !(estat & ESTAT_DISABLE_INPUT) )
	 {
	    EventNone e(AMPEVENT_DISABLED);
	    WaitEvent( e, 200 );
	 }
      }
   }

retErr:
   if( err )
      cml.Warn( "Amp %d failed init %s\n", nodeID, err->toString() );
   else
      cml.Debug( "Amp %d, init done\n", nodeID );

   return err;
}

/***************************************************************************/
/**
  Re-initialize an amplifier.  This function simply calls Amp::Init using 
  the same parameters that were initially passed.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::ReInit( void )
{
   cml.Debug( "Amp %d ReInit\n", GetNodeID() );
   if( !co ) return &CanOpenError::NotInitialized;
   return Init( *co, GetNodeID(), initialSettings );
}

/***************************************************************************/
/**
  Reset the amplifier object.  This function should be used for Amp objects 
  instead of Node::ResetNode().  It resets the amplifier and re-initializes 
  the amplifier object.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::Reset( void )
{
   cml.Debug( "Amp %d Reset\n", GetNodeID() );
   bool oldResetOnInit = initialSettings.resetOnInit;
   initialSettings.resetOnInit = true;

   const Error *err = ReInit();

   initialSettings.resetOnInit = oldResetOnInit;
   return err;
}

/***************************************************************************/
/**
Request a status update from the amplifier.  
 */
/***************************************************************************/
const Error *Amp::RequestStatUpdt( void )
{
   // Note that 8367 based products don't support remote 
   // requests well, so we use an SDO to request the PDO
   if( (hwType & 0xFF00) < 0x0300 )
      return statPdo.Request();
   else
      return sdo.Dnld8( OBJID_PDOREQUEST, 0, (uint8)0 );
}

/***************************************************************************/
/**
  Execute a home move.  The various homing parameters are passed in the
  HomeConfig structure.

  This function simply programs all the homing parameters passed in the 
  structure, then calls Amp::GoHome().

  @param cfg The homing configuration parameter structure.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GoHome( HomeConfig &cfg )
{
   /**************************************************
    * If a move is necessary, make sure that the accel
    * and velocity parameters are non-zero.
    **************************************************/
   const Error *err = 0;

   switch( cfg.method )
   {
      case CHM_HARDSTOP_POS:
      case CHM_HARDSTOP_NEG:
      case CHM_HARDSTOP_ONDX_POS:
      case CHM_HARDSTOP_ONDX_NEG:
	 err = SetHomeCurrent( cfg.current );
	 if( !err ) err = SetHomeDelay( cfg.delay );
	 // fall through

      default:
	 // If no homing move is defined, don't try to set
	 // the velocity & accel parameters.  This avoids
	 // possible errors.
	 if( (cfg.method == CHM_NONE) && (cfg.offset == 0) )
	    break;

	 // Set the home parameters
	 if( !err ) err = SetHomeVelFast( cfg.velFast );
	 if( !err ) err = SetHomeVelSlow( cfg.velSlow );
	 if( !err ) err = SetHomeAccel( cfg.accel );
   }

   if( !err ) err = SetHomeOffset( cfg.offset );
   if( !err ) err = SetHomeMethod( cfg.method, cfg.extended );

   // Start the home.
   if( !err ) err = GoHome();
   return err;
}

/***************************************************************************/
/**
  Execute a home move.  The various homing parameters (method, velocity, etc)
  are assumed to have already be configured.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GoHome( void )
{
   cml.Debug( "Amp %d Home\n", GetNodeID() );

   const Error *err;

   // Check for error conditions
   err = CheckStateForMove();
   if( err ) goto retErr;

   // Make sure we are in homing mode
   err = SetAmpMode( AMPMODE_CAN_HOMING );
   if( err ) goto retErr;

   // Clear bit 4 of the control word
   if( lastCtrlWord != 0x000F )
   {
      err = SetControlWord( 0x000F );
      if( err ) goto retErr;
   }

   // Start homing
   err = SetControlWord( 0x001F );
   if( err ) goto retErr;

   // Wait for the move to start if the
   // home method isn't NONE.
   if( lastHomeMethod != CHM_NONE )
   {
      EventNone e( AMPEVENT_MOVEDONE );
      err = WaitEvent( e, sdo.GetTimeout() );
   }

   // Otherwise, wait for the 'trajectory aborted' bit to be
   // clear if it was set.
   else
   {
      EventNone e( AMPEVENT_ABORT );
      err = WaitEvent( e, sdo.GetTimeout() );
   }

retErr:
   if( err )
      cml.Warn( "Amp %d Home failed: %s\n", GetNodeID(), err->toString() );

   return err;
}

/***************************************************************************/
/**
  Setup a point to point move, but do not start it.  The move may be subsequently
  started using Amp::StartMove().

  The move will use the trapezoidal profile mode, and all parameters will be 
  programmed based on the values passed in the cfg structure.

  @param cfg A structure holding all the move configuration parameters.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::SetupMove( ProfileConfigTrap &cfg )
{
   // Setup the move
   const Error *err = SetProfileType( PROFILE_TRAP );

   uunit dec = (cfg.dec <= 0) ? cfg.acc : cfg.dec;

   if( !err ) err = SetProfileVel( cfg.vel ); 
   if( !err ) err = SetProfileAcc( cfg.acc ); 
   if( !err ) err = SetProfileDec( dec );
   if( !err ) err = SetTargetPos( cfg.pos );

   // Make sure we are in point to point move mode
   if( !err ) err = SetAmpMode( AMPMODE_CAN_PROFILE );

   return err;
}

/***************************************************************************/
/**
  Setup a point to point move, but do not start it.  The move may be subsequently
  started using Amp::StartMove().

  The move will use the S-curve profile mode, and all parameters will be 
  programmed based on the values passed in the cfg structure.

  @param cfg A structure holding all the move configuration parameters.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::SetupMove( ProfileConfigScurve &cfg )
{
   const Error *err;

   // Make sure there is no move in progress
   if( !(eventMap.getMask() & AMPEVENT_MOVEDONE) )
      return &AmpError::InMotion;

   // Setup the move
   err = SetProfileType( PROFILE_SCURVE );

   if( !err ) err = SetProfileVel( cfg.vel ); 
   if( !err ) err = SetProfileAcc( cfg.acc ); 
   if( !err ) err = SetProfileJerk( cfg.jrk );
   if( !err ) err = SetTargetPos( cfg.pos );

   // Make sure we are in point to point move mode
   if( !err ) err = SetAmpMode( AMPMODE_CAN_PROFILE );

   return err;
}

/***************************************************************************/
/**
  Setup a point to point move, but do not start it.  The move may be subsequently
  started using Amp::StartMove().

  The move will use the velocity profile mode, and all parameters will be 
  programmed based on the values passed in the cfg structure.

  @param cfg A structure holding all the move configuration parameters.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::SetupMove( ProfileConfigVel &cfg )
{
   // Setup the move
   const Error *err = SetProfileType( PROFILE_VEL );

   if( !err ) err = SetProfileVel( cfg.vel ); 
   if( !err ) err = SetProfileAcc( cfg.acc ); 
   if( !err ) err = SetProfileDec( cfg.dec );
   if( !err ) err = SetTargetPos( cfg.dir );

   // Make sure we are in point to point move mode
   if( !err ) err = SetAmpMode( AMPMODE_CAN_PROFILE );

   return err;
}


/***************************************************************************/
/**
  Perform a point to point move.  The move will use the trapezoidal profile
  mode, and all parameters will be programmed before the move is started.

  This function can also be used to update a move that is alread in progress.

  @param cfg A structure holding all the move configuration parameters.
  @param relative This will be a relative move if true, absolute if false (default)
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::DoMove( ProfileConfigTrap &cfg, bool relative )
{
   // Setup the move
   const Error *err = SetupMove( cfg );

   // Start it
   if( !err ) err = StartMove( relative );

   return err;
}

/***************************************************************************/
/**
  Perform a point to point move.  The move will use the S-curve profile
  mode, and all parameters will be programmed before the move is started.
  @param cfg A structure holding all the move configuration parameters.
  @param relative This will be a relative move if true, absolute if false (default)
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::DoMove( ProfileConfigScurve &cfg, bool relative )
{
   // Setup the move
   const Error *err = SetupMove( cfg );

   // Start it
   if( !err ) err = StartMove( relative );

   return err;
}

/***************************************************************************/
/**
  Perform a velocity profile move.  All parameters will be programmed before 
  the move is started.

  This function can also be used to update a move that is alread in progress.

  @param cfg A structure holding all the move configuration parameters.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::DoMove( ProfileConfigVel &cfg )
{
   // Setup the move
   const Error *err = SetupMove( cfg );

   // Start it
   if( !err ) err = StartMove( false );

   return err;
}

/***************************************************************************/
/**
  Perform a point to point move.  It's assumed that the drive
  is already configured with the properly trajectory parameters (velocity, 
  acceleration, deceleration, profile type, etc).

  This function sets the trajectories target position to the passed value, 
  and manipulates the control word to start the move.  It can be used for 
  either absolute moves or relative moves

  @param pos Position to move to (absolute) or distance to move (relative).
  @param relative True if this is a relative move, false for absolute.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::DoMove( uunit pos, bool relative )
{
   const Error *err;

   // Make sure we are in point to point move mode
   err = SetAmpMode( AMPMODE_CAN_PROFILE );
   if( err ) return err;

   err = SetTargetPos( pos );
   if( err ) return err;

   return StartMove( relative );
}

/***************************************************************************/
/**
  Start the move that's already been programmed.  This function is primarily
  intended for internal use, and is called by DoMove and SendTrajectory.
  Note that the amplifier mode must have already been setup when this function
  is called.  The mode should be either AMPMODE_CAN_PROFILE, or AMPMODE_CAN_PVT.
  This function is not used to start a homing move.

  @param relative If true, start a relative move.  If false, start an absolute
         move.  Note that this is only used with point to point moves, interpolated
         moves should always set relative to false.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::StartMove( bool relative )
{
   cml.Debug( "Amp %d new move\n", GetNodeID() );
   const Error *err;

   // See if there are any errors that would stop us
   err = CheckStateForMove();
   if( err ) goto retErr;

   // Clear bit 4 of the control word
   if( lastCtrlWord != 0x000F )
      err = SetControlWord( 0x000F );

   // Wait for the amplifier to respond by clearing
   // the acknowledge bit in it's status register
   if( err ) 
      goto retErr;
   else
   {
      EventNone e( AMPEVENT_SPACK );
      err = WaitEvent( e, sdo.GetTimeout() );
      if( err ) goto retErr;
   }

   // Start the move
   if( relative )
      err = SetControlWord( 0x007F );
   else
      err = SetControlWord( 0x003F );

   // Now, wait for the acknowledgement
   if( !err )
   {
      EventAny e( AMPEVENT_SPACK );
      err = WaitEvent( e, sdo.GetTimeout() );
   }

retErr:
   if( err )
      cml.Warn( "Amp %d move failed: %s\n", GetNodeID(), err->toString() );

   return err;
}

/***************************************************************************/
/**
  Wait for the currently running move to finish, or for an error to occur.

  @param timeout The maximum time to wait (milliseconds)
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitMoveDone( int32 timeout )
{
   EventAny e( AMPEVENT_MOVEDONE | AMPEVENT_NODEGUARD | AMPEVENT_FAULT | 
      AMPEVENT_ERROR | AMPEVENT_DISABLED | AMPEVENT_QUICKSTOP |
      AMPEVENT_ABORT );

   // Wait for the event
   const Error *err = WaitEvent( e, timeout );

   // Return an error code based on the events that occurred.
   if( !err ) err = GetErrorStatus();

   return err;
}

/***************************************************************************/
/**
  Wait for an amplifier event condition. This function can be used to wait
  on any generic event associated with the amplifier.  
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match Returns the matching event condition.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitEvent( Event &e, int32 timeout, AMP_EVENT &match )
{
   // We have the amp programmed to only send status updates when something
   // changes.  This could lead to an incorrect timeout of a status message
   // was lost.  I'll try to avoid this here by occasionally polling the 
   // status if the event isn't satisfied.
   //
   // I'll try to find a reasonable polling interval based on the passed timeout
   int32 poll;

   if( timeout < 0 )
      poll = 500;
   else
   {
      if( timeout < 10 )
	 poll = timeout;
      else if( timeout < 1000 )
	 poll = timeout/2+1;
      else
	 poll = 500;
      timeout -= poll;
   }

   const Error *err;
   do
   {
      err = e.Wait( eventMap, poll );
      match = (AMP_EVENT)e.getMask();

      if( err != &ThreadError::Timeout )
	 return err;

      if( poll )
	 RequestStatUpdt();

      // Find remaining timeout after this wait
      if( timeout >= 0 )
      {
	 if( timeout < poll )
	    poll = timeout;

	 timeout -= poll;
      }
   } while( poll ); 

   return err;
}

/***************************************************************************/
/**
  Wait for an amplifier event condition. This function can be used to wait
  on any generic event associated with the amplifier.  
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever (default).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitEvent( Event &e, int32 timeout )
{
   AMP_EVENT match;
   return WaitEvent( e, timeout, match );
}


/***************************************************************************/
/**
  Wait on the amplifier's general purpose input pins.

  The amplifier object maintains an EventMap object which reflects the state
  of the amplifier's general purpose input pins.  Each bit of this EventMap
  corresponds to one input pin; bit 0 for input 0, bit 1 for input 1, etc.
  The bit in the event map is set when the corresponding input pin is high,
  and cleared when the input pin is low.

  This function provides a very flexible method for waiting on a particular
  state on the input pins.  Event objects may be created to define a specific
  state of one or more pins, and these objects may be used in conjunction with
  this function to wait for that state to occur.

  In addition to this function, two simpler functions are also provided.  These
  functions (WaitInputHigh and WaitInputLow) allow the user to wait on one or 
  more input pins to go high or low respectively.  Internally, these function 
  call WaitInputEvent for their implementation.

  @param e       An Event object describing the input pin state to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match   On success, the state of the input pins which caused the match 
  to occur will be returned here.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitInputEvent( Event &e, int32 timeout, uint32 &match )
{
   const Error *err = e.Wait( inputStateMap, timeout );
   match = e.getMask();
   return err;
}

/***************************************************************************/
/**
  Wait for any of the specified general purpose input pins to be set.  The
  inputs parameter specifies which input(s) to wait on using a bit mask.  Bit
  0 should be set for input 0, bit 1 for input 1, etc.  The function will 
  return when any of the specified input pins goes high.
  @param inputs Specifies which input pin(s) to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.  If not specified, the timeout defaults to -1
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitInputHigh( uint32 inputs, int32 timeout )
{
   EventAny e( inputs );
   uint32 match;

   return WaitInputEvent( e, timeout, match );
}

/***************************************************************************/
/**
  Wait for any of the specified general purpose input pins to be lowered.  The
  inputs parameter specifies which input(s) to wait on using a bit mask.  Bit
  0 should be set for input 0, bit 1 for input 1, etc.  The function will 
  return when any of the specified input pins goes low.
  @param inputs Specifies which input pin(s) to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.  If not specified, the timeout defaults to -1
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::WaitInputLow( uint32 inputs, int32 timeout )
{
   EventAnyClear e( inputs );
   uint32 match;

   return WaitInputEvent( e, timeout, match );
}

/***************************************************************************/
/**
  Get the current state of the amplifier's event mask.  The event mask is a 
  bit-mapped variable identifies many interesting elements of the amplifiers
  state.  The contents of this variable are built up from several different
  amplifier status words which are constantly updated over the CANopen network.

  When the event mask is read using this function, no new messages are passed
  over the network.  The current value of the event mask is simply returned.
  Any time the amplifier's state changes, it sends a message over the CANopen
  network which is used to update this mask.

  It is also possible to wait on a particular value for this mask.  See
  Amp::WaitEvent for details.

  @param e The amplifier's event mask is returned here
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GetEventMask( AMP_EVENT &e )
{
   e = (AMP_EVENT)eventMap.getMask();
   return 0;
}

/***************************************************************************/
/**
  Return an error object identifying the amplifiers status.  This function 
  causes the amplifier object to examine it's event mask and return an 
  error object corresponding to the most serious error present.
  @param noComm If true, then no CAN message communications will be performed
  by this function.  This is useful if the function is being called from
  a CAN message handler which can't perform SDO communications.
  If false (default), then the amplifier may be queried for more detailed
  error information.
  @return A pointer to an error object, or NULL if no errors are present
  */
/***************************************************************************/
const Error *Amp::GetErrorStatus( bool noComm )
{
   uint32 events = eventMap.getMask();
   EVENT_STATUS stat = (EVENT_STATUS)statPdo.estat.Read();
   const Error *err = 0;

   if( events & AMPEVENT_NODEGUARD )
      err = &NodeError::GuardTimeout;

   else if( events & AMPEVENT_FAULT )
   {
      // Get detailed fault information if CAN communications are allowed
      if( !noComm )
      {
	 AMP_FAULT faults;
	 GetFaults( faults );
	 err = AmpFault::DecodeFault( faults );
      }

      // If no error was found, look for one in the event status
      if( !err )
	 err = AmpError::DecodeStatus( stat );

      // If all else fails, return a generic error
      if( !err )
	 err = &AmpError::Unknown;
   }

   else if( events & AMPEVENT_ERROR )
   {
      err = AmpError::DecodeStatus( stat );
      if( !err ) err = &AmpError::Unknown;
   }

   else if( events & AMPEVENT_QUICKSTOP )
      err = &AmpError::QuickStopMode;

   else if( events & AMPEVENT_ABORT )
      err = &AmpError::Abort;

   else if( events & AMPEVENT_DISABLED )
      err = &AmpError::Disabled;

   return err;
}

/***************************************************************************/
/**
  Handle an amplifier state change.  This method wakes up any task waiting on
  the move done semaphore in the event of a guard error.  If this feature is
  desired, this method should be called from any class that over rides this
  method.
  */
/***************************************************************************/
void Amp::HandleStateChange( NodeState from, NodeState to )
{
   // On a guard error, wake up any task that's pending
   // on my semaphore (i.e. waiting for move done, etc)
   if( to == NODESTATE_GUARDERR )
      eventMap.setBits( AMPEVENT_NODEGUARD );
}

/***************************************************************************/
/**
  This function attempts to clear a node guarding event condition.  Node guarding
  events occur when the amplifier fails to respond to it's heartbeat protocol
  for some reason.  This could be caused by a network wiring problem, slow 
  processing on the master controller (causing the amplifier guard message to be
  delayed or lost), or an amplifier error such as a reset or power down.

  In any case, once a node guarding error is identified, the error condition 
  must be cleared before any new moves may be performed.

  This function attempts to clear the node guarding event condition, however if
  it determines that the amplifier has been reset then it fails and returns the
  error object AmpError::Reset.  In this case, the amplifier object must be 
  reinitialized before it can be used.  The amp may be reinitialized by calling
  Amp::Init or Amp::ReInit.

  If node guarding error become a problem, it may mean that the guard time is 
  set too low.  This can be adjusted when the amplifier object is initialized
  by the values in the AmpSettings object.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::ClearNodeGuardEvent( void )
{
   // Check the latched event status to determine if the amp was reset
   EVENT_STATUS status;
   const Error *err = GetEventLatch( status );
   if( err ) return err;

   if( status & ESTAT_RESET )
      return &AmpError::Reset;

   eventMap.clrBits( AMPEVENT_NODEGUARD );
   return 0;
}

/***************************************************************************/
/**
  Check the amplifier's state to make sure a move can be started.  This 
  function is used internally by the functions that start moves & homing.
  It looks at the current state of the amplifier and returns an appropriate
  error code if something is wrong that would cause problems during a move/home.
  @return A pointer to an error object, or NULL for no error
  */
/***************************************************************************/
const Error *Amp::CheckStateForMove( void )
{
   // Make sure the node is operational
   if( GetState() != NODESTATE_OPERATIONAL )
      return &AmpError::NodeState;

   if( !enabled )
      return &AmpError::Disabled;

   uint32 mask = eventMap.getMask();

   // First, just look for obvious indications of an error.  If none
   // are found, then I'm good to go.
   if( !(mask & (AMPEVENT_NODEGUARD|AMPEVENT_FAULT|AMPEVENT_ERROR|
	       AMPEVENT_DISABLED|AMPEVENT_QUICKSTOP) ) )
      return 0;

   // Return a node guarding error if that was detected.
   if( mask & AMPEVENT_NODEGUARD ) return &AmpError::GuardError;

   // If the amplifier is in a fault state, return details
   if( mask & AMPEVENT_FAULT )
   {
      AMP_FAULT faults;
      GetFaults( faults );
      return AmpFault::DecodeFault( faults );
   }

   if( mask & AMPEVENT_QUICKSTOP ) return &AmpError::QuickStopMode;

   if( mask & AMPEVENT_DISABLED ) return &AmpError::Disabled;

   // Otherwise, return the error info.  If no errors are detected, 
   // then they may have been cleared recently.
   uint32 event = statPdo.estat.Read();

   event &= (ESTAT_SHORT_CRCT | ESTAT_AMP_TEMP | ESTAT_OVER_VOLT | 
	 ESTAT_UNDER_VOLT | ESTAT_MTR_TEMP | ESTAT_ENCODER_PWR | 
	 ESTAT_PHASE_ERR | ESTAT_TRK_ERR);

   return AmpError::DecodeStatus( (EVENT_STATUS)event );
}

/***************************************************************************/
/**
  Decode the passed event status word and return an appropriate error object.
  @param stat The amplifier event status register
  @return A pointer to an error object, or NULL if there is no error.
  */
/***************************************************************************/
const AmpError *AmpError::DecodeStatus( EVENT_STATUS stat )
{
   if( stat & ESTAT_SHORT_CRCT  ) return &AmpError::ShortCircuit;
   if( stat & ESTAT_AMP_TEMP    ) return &AmpError::AmpTemp;
   if( stat & ESTAT_MTR_TEMP    ) return &AmpError::MotorTemp;
   if( stat & ESTAT_ENCODER_PWR ) return &AmpError::EncoderPower;
   if( stat & ESTAT_PHASE_ERR   ) return &AmpError::PhaseErr;
   if( stat & ESTAT_TRK_ERR     ) return &AmpError::TrackErr;
   if( stat & ESTAT_OVER_VOLT   ) return &AmpError::OverVolt;
   if( stat & ESTAT_UNDER_VOLT  ) return &AmpError::UnderVolt;
   if( stat & ESTAT_FAULT       ) return &AmpError::Fault;
   if( stat & ESTAT_POSLIM      ) return &AmpError::PosLim;
   if( stat & ESTAT_NEGLIM      ) return &AmpError::NegLim;   
   if( stat & ESTAT_SOFTLIM_POS ) return &AmpError::PosSoftLim;
   if( stat & ESTAT_SOFTLIM_NEG ) return &AmpError::NegSoftLim;
   if( stat & ESTAT_TRK_WARN    ) return &AmpError::TrackWarn;

   return 0;
}

/***************************************************************************/
/**
  Return an appropriate fault object based on the amplifier fault mask.
  @param fault The amplifier fault mask.
  @return A pointer to the fault object, NULL if there is no fault.
  */
/***************************************************************************/
const AmpFault *AmpFault::DecodeFault( AMP_FAULT fault )
{
   if( fault == 0 ) return 0;

   if( fault & FAULT_DATAFLASH   ) return &AmpFault::Memory;
   if( fault & FAULT_ADCOFFSET   ) return &AmpFault::ADC;
   if( fault & FAULT_SHORT_CRCT  ) return &AmpFault::ShortCircuit;
   if( fault & FAULT_AMP_TEMP    ) return &AmpFault::AmpTemp;
   if( fault & FAULT_MTR_TEMP    ) return &AmpFault::MotorTemp;
   if( fault & FAULT_OVER_VOLT   ) return &AmpFault::OverVolt;
   if( fault & FAULT_UNDER_VOLT  ) return &AmpFault::UnderVolt;
   if( fault & FAULT_ENCODER_PWR ) return &AmpFault::EncoderPower;
   if( fault & FAULT_PHASE_ERR   ) return &AmpFault::PhaseErr;
   if( fault & FAULT_TRK_ERR     ) return &AmpFault::TrackErr;
   if( fault & FAULT_I2T_ERR     ) return &AmpFault::I2TLimit;

   return &AmpFault::Unknown;
}

/***************************************************************************/
/**
  This is a simple local function that checks an AMP_MODE to determine if it's
  a true CAN control mode or not.
  */
/***************************************************************************/
static bool isCanMode( AMP_MODE mode )
{
   switch( mode & 0xFF00 )
   {
      case 0:
	 return (mode==0) ? false : true;

      case AMPMODE_CAN_SERVO:
      case AMPMODE_CAN_USTEP:
	 return true;

      default:
	 return false;
   }
}

/***************************************************************************/
/**
  Set the amplifier mode of operation.  The mode of operation determines the
  top level control loop that will be controled (position, velocity, or current),
  and the source of that control (CANopen network, digital input pins, etc).

  @param mode The mode of operation to be set
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::SetAmpMode( AMP_MODE mode )
{
   const Error *err = 0;

   // For CAN modes, if the control method (servo or microstep) isn't
   // specified, then fill it in.
   if( isCanMode(mode) && !(mode&0xFF00) )
      mode = (AMP_MODE)(mode | canCtrlMethod);

   // Don't bother updating the mode if it hasn't changed.
   if( lastMode == mode ) return 0;

   cml.Debug( "Amp %d new mode 0x%04x\n", GetNodeID(), mode );

   // If this is a CAN control mode, then I need to set both
   // the amplifier's operating mode & the CAN mode.
   // I don't have to worry about enable/disable issues though.
   if( isCanMode( mode ) )
   {
      // Only set the amplifier desired state if it's changed
      if( 0xFF00 & (lastMode^mode) )
      {
	 uint16 i = ((uint16)mode)>>8;
	 err = sdo.Dnld16( OBJID_AMP_MODE, 0, i );
      }

      if( !err ) err = sdo.Dnld8( OBJID_OP_MODE, 0, ByteCast(mode) );

      // Save the control method for next time
      if( !err ) canCtrlMethod = (AMP_MODE)(mode & 0xFF00);
   }

   // For other modes, if the amp is disabled then I need to set the mode
   // to zero (which does the disable).  The actual mode will be set when
   // the amp is enabled.
   else if( !enabled )
      err = sdo.Dnld16( OBJID_AMP_MODE, 0, (int16)0 );

   // If I'm enabled, then just set the mode and return.
   else
   {
      uint16 i = ((uint16)mode)>>8;
      err = sdo.Dnld16( OBJID_AMP_MODE, 0, i );
   }

   if( !err ) lastMode = mode;

   return err;
}

/***************************************************************************/
/**
  Get the currently active amplifier mode of operation.
  @param mode The active mode of operation is returned here
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GetAmpMode( AMP_MODE &mode )
{
   mode = lastMode;
   return 0;
}

/***************************************************************************/
/**
  Perform a 'quick stop' on the axis.  The exact meaning of a quick stop 
  can be programmed using the Amp::SetQuickStop function.  
  Regardless of the type of quick stop being performed, the amplifier will
  always end up disabled at the end of the quick stop.  If disabling the 
  amplifier is not desirable, then the Amp::HaltMove function should be 
  used instead.

  Note that the quick stop function is only available when running in one
  of the standard CAN amplifier modes.  If doing low level velocity or current
  control, then moves must be stopped externally.
  */
/***************************************************************************/
const Error *Amp::QuickStop( void )
{
   const Error *err;

   err = SetControlWord( 0x0003 );
   if( err ) return err;

   // Now, wait for the amp to either become disabled, 
   // or indicate that it's doing a quick stop.  One
   // or the other should happen depending on the 
   // quick stop mode that's selected
   EventAny e( AMPEVENT_QUICKSTOP | AMPEVENT_DISABLED );
   return WaitEvent( e, sdo.GetTimeout() );
}

/***************************************************************************/
/**
  Halt the current move.  The exact type of halt can be programmed using 
  the Amp::SetHaltMode function.

  Note that the halt function is only available when running in one of the 
  standard CAN amplifier modes.  If doing low level velocity or current
  control, then moves must be stopped externally.
  */
/***************************************************************************/
const Error *Amp::HaltMove( void )
{
   return SetControlWord( lastCtrlWord | 0x0100 );
}

/***************************************************************************/
/**
  Disable the amplifier. 
  Note that if the brake delays are in use, then the amplifier may still be
  enabled when this function returns success.  The outputs will actually be
  disabled after the amplifier finishes the braking procedure.

  @param wait Wait for confirmation from the amplifier if true (default).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::Disable( bool wait )
{
   const Error *err = 0;

   // If we're not in a CAN controlled mode, then we need
   // to set the mode to 0 to disable
   if( !isCanMode( lastMode ) )
      err = sdo.Dnld16( OBJID_AMP_MODE, 0, (int16)0 );

   // In normal CAN controlled modes, I handle this
   // through the use of the control register.
   // Note that I clear bit 1 to get out of quick stop mode
   else
      err = SetControlWord( 0x0005 );
   if( err ) return err;

   enabled = false;

   if( !wait ) return 0;

   // Now, wait for the amp to actually disable before returning
   // Note that when this returns the amplifier is trying to
   // disable, but if the motor's brake times are set, then this
   // could take a long time.
   EventAny e( AMPEVENT_DISABLED | AMPEVENT_SOFTDISABLE );
   return WaitEvent( e, sdo.GetTimeout() );
}

/***************************************************************************/
/**
  Enable the amplifier.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::Enable( bool wait )
{
   // First, check to see if the amplifier is in a fault state.
   // If it is, then it will ignore my enable request causing a
   // timeout error when I wait for the state change below.
   if( eventMap.getMask() & AMPEVENT_FAULT )
   {
      AMP_FAULT faults;
      GetFaults( faults );
      return AmpFault::DecodeFault( faults );
   }

   const Error *err;

   // Check for other errors which would prevent us from enabling
   if( eventMap.getMask() & AMPEVENT_ERROR )
   {
      EVENT_STATUS stat = (EVENT_STATUS)statPdo.estat.Read();
      err = AmpError::DecodeStatus( stat );
      if( err ) return err;
   }

   // If we're not in a CAN controlled mode, then we need
   // update the amplifier mode to enable it.
   enabled = true;

   if( !isCanMode( lastMode ) )
   {
      AMP_MODE newMode = lastMode;
      lastMode = AMPMODE_DISABLED;
      err = SetAmpMode( newMode );
      if( err ) return err;
      lastMode = newMode;
   }

   // In normal CAN controlled modes, I handle this
   // through the use of the control register.
   else
   {
      err = SetControlWord( 0x000F );
      if( err ) return err;
   }

   if( !wait ) return 0;

   // Now, wait for the amp to actually enable before returning
   EventNone e( AMPEVENT_SOFTDISABLE );
   err = WaitEvent( e, sdo.GetTimeout() );

   // A timeout here could mean that a fault condition occurred
   // between the time I checked above and when I sent the 
   // new control word.  
   if( err == &ThreadError::Timeout )
   {
      if( eventMap.getMask() & AMPEVENT_FAULT )
      {
	 AMP_FAULT faults;
	 GetFaults( faults );
	 return AmpFault::DecodeFault( faults );
      }
   }

   return err;
}

/***************************************************************************/
/**
  Return true if the amplifier's PWM outputs are currently enabled.
  @return true if the amplifier's PWM outputs are currently enabled.
  */
/***************************************************************************/
bool Amp::IsHardwareEnabled( void )
{
   return !(eventMap.getMask() & AMPEVENT_DISABLED);
}

/***************************************************************************/
/**
  Return true if the amplifier is being enabled by software.  The amplifier
  outputs may still be disabled if this is true due to an error condition, etc.
  @return true if the amplifier is enabled by software.
  */
/***************************************************************************/
bool Amp::IsSoftwareEnabled( void )
{
   return enabled;
}

/***************************************************************************/
/**
  Return true if the amplifier has been successfully referenced (homed).

  When an amplifier is first powered up (or after a reset) it does not know
  the absolute position of the motor.  Once the home routine has been 
  successfully executed, the encoder zero location is known and the amplifier
  is considered referenced.

  Once an amplifier has been referenced, it will not loose reference until it
  is reset, or until a new home routine is executed.  During the execution of
  a home routine, the amplifier is considered to be unreferenced.  If the 
  home routine is completed successfully, the amplifier will then be referenced
  again.

  @return true if the amplifier has been referenced.  Return false if the 
  amplifier has not been referenced if an error occurs reading this 
  information from the amplifier.
  */
/***************************************************************************/
bool Amp::IsReferenced( void )
{
   uint16 trjStatus;
   if( sdo.Upld16( OBJID_TRJ_STATUS, 0, trjStatus ) )
      return false;

   return (trjStatus & 0x1000) == 0x1000;
}

/***************************************************************************/
/**
  Default constructor for amplifier settings object.  This constructor sets
  all the settings to the default values.
  */
/***************************************************************************/
AmpSettings::AmpSettings( void )
{
   // Default to 10 ms synch period using the standard ID (0x80)
   synchPeriod      = 10000;
   synchID          = 0x00000080;
   synchProducer    = false;
   synchUseFirstAmp = true;
   timeStampID      = 0x00000180;

   // Default to using node guarding with a 200 ms guard time.
   heartbeatPeriod  = 0;
   heartbeatTimeout = 200;
   guardTime      = 200;
   lifeFactor       = 3;

   // By default, we put the amp into homing mode and
   // enable it at init time.
   enableOnInit = true;
   initialMode = AMPMODE_CAN_HOMING;

   resetOnInit = false;

   maxPvtSendCt = 6;
}

