/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
  Implementation of the Linkage class.
  */

#include "CML.h"

// Note, This disables an annoying VC++ warning
#ifdef _WIN32
#pragma warning( disable: 4355 )
#endif

CML_NAMESPACE_USE();

// Linkage errors
CML_NEW_ERROR( LinkError, BadAmpCount,      "An illegal number of amplifiers was passed to Linkage::Init" );
CML_NEW_ERROR( LinkError, NetworkMismatch,  "The amplifiers passed to Linkage::Init don't share a CanOpen network" );
CML_NEW_ERROR( LinkError, AlreadyInit,      "The linkage is already initialized" );
CML_NEW_ERROR( LinkError, AmpAlreadyLinked, "The passed amplifier object is already assigned to a linkage" );
CML_NEW_ERROR( LinkError, AxisCount,        "The point dimension doesn't match the number of linkage axes" );
CML_NEW_ERROR( LinkError, AmpTrjOverflow,   "Amplifier trajectory structure overflow." );
CML_NEW_ERROR( LinkError, AmpTrjInUse,      "Amplifier trajectory already in use" );
CML_NEW_ERROR( LinkError, AmpTrjNotRunning, "Amplifier trajectory not presently in use" );
CML_NEW_ERROR( LinkError, NoActiveTrj,      "No linkage trajectory is active" );
CML_NEW_ERROR( LinkError, BadMoveLimit,     "A zero or negative move limit was detected" );
CML_NEW_ERROR( LinkError, UnknownAmpErr,    "An amplifier error occurred" );
CML_NEW_ERROR( LinkError, StartMoveTO,      "Timeout waiting on amplifier to respond to start move command" );
CML_NEW_ERROR( LinkError, NotSupported,     "Support for this function was not enabled in the library" );

/***************************************************************************/
/**
  Default constructor.  Linkage::Init must be called before this linkage 
  object may be used.
  */
/***************************************************************************/
Linkage::Linkage( void ): ctrlPDO( *this )
{
   trjUseCount = 0;
   ampct = 0;
   maxVel = maxAcc = maxDec = maxJrk = 0;
   linkTrjPtr = 0;

   for( int i=0; i<CML_MAX_AMPS_PER_LINK; i++ )
   {
      amp[i] = 0;
#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
      ampTrj[i].Init( this );
#endif
   }

   ClearLatchedError();
}

/***************************************************************************/
/**
  Linkage object destructor.
  */
/***************************************************************************/
Linkage::~Linkage()
{
   for( int i=0; i<ampct; i++ )
   {
      if( amp[i] ) amp[i]->SetLinkage(0);
   }
}

/***************************************************************************/
/**
  Mark the specified Amp as invalid.  This is called by an Amp object during
  destruction if the amp is still attached to a linkage (which is an error).
  @param a Pointer to the amp
  */
/***************************************************************************/
void Linkage::InvalidateAmp( Amp *a )
{
   for( int i=0; i<ampct; i++ )
   {
      if( amp[i] == a ) amp[i] = 0;
   }
   return;
}

/***************************************************************************/
/**
  Configure a linkage.  The linkage object will be configured to use the 
  various settings passed in the LinkSettings object.

  When a new Linkage object is created, it will be configured using a 
  default set of settings.  These settings can be modified through this
  call.  Set the documentation of the LinkSettings object for details of 
  the available settings and their default values.

  @param settings The new settings to be used.  A local copy of this object
  will be made by the linkage.
  */
/***************************************************************************/
const Error *Linkage::Configure( LinkSettings &settings )
{
   cfg = settings;
   return 0;
}

/***************************************************************************/
/**
  Initialize a new linkage object.  If the object has already been initialized,
  this will fail with an error.

  All amplifiers attached to a linkage must be initialized, and must share the
  same CanOpen network object.  Also, amplifiers may only be attached to one
  linkage at a time, so this function will fail if any of the passed amplifier
  objects is already attached to a Linkage.

  The linkage object will maintain pointers to each of the amplifier objects
  passed to this function.  The amplifiers may not be destroyed until after 
  the linkage object is.  

  @param ct The number of amplifiers to be used with this linkage.
  Note that this must be between 1 and CML_MAX_AMPS_PER_LINK.

  @param a An array of amplifiers to be assigned to this linkage.  There
  must be at least ct amplifiers in this array.
  */
/***************************************************************************/
const Error *Linkage::Init( uint16 ct, Amp a[] )
{
   if( ct < 1 || ct > CML_MAX_AMPS_PER_LINK )
      return &LinkError::BadAmpCount;

   Amp *aptr[ CML_MAX_AMPS_PER_LINK ];

   for( int i=0; i<ct; i++ )
      aptr[i] = &a[i];

   return Init( ct, aptr );
}

/***************************************************************************/
/**
  Initialize a new linkage object.  If the object has already been initialized,
  this will fail with an error.

  All amplifiers attached to a linkage must be initialized, and must share the
  same CanOpen network object.  Also, amplifiers may only be attached to one
  linkage at a time, so this function will fail if any of the passed amplifier
  objects is already attached to a Linkage.

  The linkage object will maintain pointers to each of the amplifier objects
  passed to this function.  The amplifiers may not be destroyed until after 
  the linkage object is.  

  @param ct The number of amplifiers to be used with this linkage.
  Note that this must be between 1 and CML_MAX_AMPS_PER_LINK.

  @param a An array of pointer to amplifier objects to be assigned to this 
  linkage.  There must be at least ct pointers in this array.
  */
/***************************************************************************/
const Error *Linkage::Init( uint16 ct, Amp *a[] )
{
   cml.Debug( "Initializing linkage %d\n", a[0]->GetNodeID() );

   CML_ASSERT( ct > 0 );
   CML_ASSERT( ct <= CML_MAX_AMPS_PER_LINK );
   CML_ASSERT( CML_MAX_AMPS_PER_LINK <= 32 );
   CML_ASSERT( amp[0] == 0 );

   if( ct < 1 || ct > CML_MAX_AMPS_PER_LINK )
      return &LinkError::BadAmpCount;

   if( amp[0] )
      return &LinkError::AlreadyInit;

   ClearLatchedError();

   // Make sure all amps are initialized, and share
   // the same network.
   int i;
   CanOpen *co = &a[0]->GetCanOpen();

   for( i=0; i<ct; i++ )
   {
      const Error *err = 0;

      if( !a[i]->IsInitialized() ) 
	 err = &CanOpenError::NotInitialized;

      else if( &(a[i]->GetCanOpen()) != co )
	 err = &LinkError::NetworkMismatch;

      else if( a[i]->GetLinkage() != 0 )
	 err = &LinkError::AmpAlreadyLinked;

      if( err )
	 return LatchError( err, i );
   }

   // Assign all the amplifiers to this linkage
   ampct = ct;
   for( i=0; i<ct; i++ )
   {
      amp[i] = a[i];
      amp[i]->SetLinkage(this);
   }

   // Add my state events to each amplifier
   for( i=0; i<ct; i++ )
   {
      stateEvent[ i ].link = this;
      amp[i]->eventMap.Add( &stateEvent[i] );
   }

   // Start my thread
   start();

   // Initialize a PDO used to send control words to each amplifier
   return ctrlPDO.Init();
}

/***************************************************************************/
/**
  Get a reference to the amplifier object at the specified location in the
  linkage.  Note that if CML_DEBUG_ASSERT is defined, then the standard C 
  assert function will be used to check for an invalid index.

  @param i The index of the amplifier to access.
  @return A reference to the amplifier object.
  */
/***************************************************************************/
Amp &Linkage::GetAmp( uint16 i )
{
   CML_ASSERT( i < ampct );  
   CML_ASSERT( amp[i] != 0 );
   return *amp[i];
}

/***************************************************************************/
/**
  Get the current commanded position of the linkage.  Note that this function
  queries the position of each amplifier sequentially and therefore the returned
  position information will only be accurate if the linkage is at rest when the
  function is called.

  @param p A point that will be filled in with the current Linkage commanded 
  position.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::GetPositionCommand( PointN &p )
{
   int axes = GetAxesCount();
   CML_ASSERT( axes <= CML_MAX_AMPS_PER_LINK );

   if( p.getDim() != axes )
      return &LinkError::AxisCount;

   uunit pos[CML_MAX_AMPS_PER_LINK];

   int i;
   const Error *err;

   for( i=0; i<ampct; i++ )
   {
      err = amp[i]->GetPositionCommand( pos[i] );
      if( err )
      {
	 LatchError( err, i );
	 return err;
      }
   }

   err = ConvertAmpToAxisPos( pos );
   if( err )
   {
      LatchError( err, -1 );
      return err;
   }

   for( i=0; i<axes; i++ )
      p[i] = pos[i];

   return 0;
}

/***************************************************************************/
/**
  Set limits used for multi-axis point-to-point moves.

  @param vel Maximum velocity
  @param acc Maximum acceleration
  @param dec Maximum deceleration
  @param jrk Maximum jerk
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::SetMoveLimits( uunit vel, uunit acc, uunit dec, uunit jrk )
{
   if( vel <= 0 || acc <= 0 || dec <= 0 || jrk <= 0 )
      return 0;

   maxVel = vel;
   maxAcc = acc;
   maxDec = dec;
   maxJrk = jrk;

   return 0;
}

/***************************************************************************/
/**
  Return the move limits currently set for this linkage.
  @param vel Returns maximum velocity
  @param acc Returns maximum acceleration
  @param dec Returns maximum deceleration
  @param jrk Returns maximum jerk
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::GetMoveLimits( uunit &vel, uunit &acc, uunit &dec, uunit &jrk )
{
   vel = maxVel;
   acc = maxAcc;
   dec = maxDec;
   jrk = maxJrk;
   return 0;
}

/***************************************************************************/
/**
  Move to a specified position.  This move uses the limits previously set using
  Linkage::SetMoveLimits.

  @param p The point to move to.
  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::MoveTo( PointN &p, bool start )
{
   return MoveTo( p, maxVel, maxAcc, maxDec, maxJrk, start );
}

/***************************************************************************/
/**
  Move to a point in space.  The number of dimensions of the point must equal
  the number of axes controlled by the Linkage (as returned by Linkage::GetAxesCount).

  This method causes the linkage to perform a straight line move in N space 
  from the present position to the specified point.  The move will be limited
  in velocity, acceleration & jerk to the passed values.

  The linkage is assumed to be at rest when this method is called.  If this isn't 
  the case, then an error will result.

  Note that this function causes a trajectory to be calculated and passed to the 
  amplifiers as a series of PVT points.  This calculation requires floating point
  math, so this function is not available if floating point support has not been 
  enabled in CML_Settings.h.

  @param p The point in N space to move to.
  @param vel Maximum velocity
  @param acc Maximum acceleration
  @param dec Maximum deceleration
  @param jrk Maximum jerk
  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::MoveTo( PointN &p, uunit vel, uunit acc, uunit dec, uunit jrk, bool start )
{
#ifndef CML_ALLOW_FLOATING_POINT
   return &LinkError::NotSupported;
#else
   ClearLatchedError();

   if( p.getDim() != GetAxesCount() )
      return &LinkError::AxisCount;

   const Error *err;

   Point<CML_MAX_AMPS_PER_LINK> startPos;
   startPos.setDim( GetAxesCount() );

   err = GetPositionCommand( startPos );
   if( err ) return err;

   err = scurve.Calculate( startPos, p, vel, acc, dec, jrk );
   if( err ) return err;

   return SendTrajectory( scurve, start );
#endif
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Upload a multi-axis PVT move trajectory to the linkage and optionally start the move.

  @param trj Reference to the linkage trajectory to be used.  A local
  pointer to this trajectory will be stored if the entire profile will 
  not fit in the amplifiers on-board buffer.  This pointer will be kept
  until the entire profile has been uploaded to the linkage.  It is therefore
  important to ensure that the trajectory object passed here will remain
  valid (i.e. not be deallocated) until the linkage has called the LinkTrajectory.Finish()
  method.

  @param start If true (the default), the profile will be started by this call.
               If false, the profile will be uploaded, but not started.  In that case
               the move may be later started by a call to Linkage::StartMove.

  @return An error object.
  */
/***************************************************************************/
const Error *Linkage::SendTrajectory( LinkTrajectory &trj, bool start )
{
   ClearLatchedError();

   // Save a reference to the passed trajectory and increase it's usage 
   // counter.  We increase the usage counter here so that the trajectory
   // won't be freed if the first amp send's it all down.
   linkTrjPtr = &trj;
   IncTrjUseCount();

   for( int i=0; i<ampct; i++ )
   {
      const Error *err = amp[i]->SendTrajectory( ampTrj[i], false );
      if( err )
      {
	 DecTrjUseCount();
	 return LatchError( err, i );
      }
   }

   // Lower the usage count here to make up for the initial increase.
   // This may cause the trajectory to be freed if it's been completely 
   // downloaded to all axes.
   DecTrjUseCount();

   if( start )
      return StartMove();

   return 0;
}
#endif

/***************************************************************************/
/**
  Start the moves that have already been programmed into all
  axes of this linkage.

  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::StartMove( void )
{
   ClearLatchedError();

   uint32 allAmps = (1<<ampct) - 1;
   const Error *err = 0;
   EventAny events[CML_MAX_AMPS_PER_LINK];
   EventMap map;
   int i;

   EventNone none( allAmps );
   EventAll all( allAmps );

   // Make sure all amps are in profile mode
   // and are ready to start a new move.
   for( i=0; i<ampct; i++ )
   {
      err = amp[i]->CheckStateForMove();
      if( err )
      {
	 err = LatchError( err, i );
	 goto cleanup;
      }

      events[i].setChain( map, (1<<i) );
      events[i].setValue( AMPEVENT_SPACK | AMPEVENT_PVT_EMPTY );
      amp[i]->eventMap.Add( &events[i] );
   }

   // Enable the amps, and wait for them to clear
   // the acknowledge bit.
   SetControlWord( 0x000F );
   err = none.Wait( map, cfg.moveAckTimeout );

   // On timeout, find the offending thread
   if( err == &ThreadError::Timeout )
   {
      for( i=0; i<ampct; i++ )
      {
	 AMP_EVENT e;
	 amp[i]->GetEventMask( e );
	 if( e & AMPEVENT_SPACK )
	 {
	    err = LatchError( &LinkError::StartMoveTO, i );
	    break;
	 }
      }
   }

   if( err )
   {
      err = LatchError( err, -1 );
      goto cleanup;
   }

   // Now, start a move on all amps, and wait for
   // the acknowledge bit to be set.
   SetControlWord( 0x003F );
   err = all.Wait( map, cfg.moveAckTimeout );

   // On timeout, find the offending thread
   if( err == &ThreadError::Timeout )
   {
      for( i=0; i<ampct; i++ )
      {
	 AMP_EVENT e;
	 amp[i]->GetEventMask( e );
	 if( !(e & AMPEVENT_SPACK) )
	 {
	    err = LatchError( &LinkError::StartMoveTO, i );
	    break;
	 }
      }
   }

   if( err )
   {
      err = LatchError( err, -1 );
      goto cleanup;
   }

   // Notify my thread that a move has started
   startSema.Put();

   // Reset the event chaining that I setup.  This ensures that the
   // events won't point to an event map that has been destroyed
   // already.
cleanup:
   for( i=0; i<ampct; i++ )
      events[i].delChain();

   return err;
}

/***************************************************************************/
/**
  Halt the current move.  The exact type of halt can be programmed individually 
  for each axis using the Amp::SetHaltMode function.
  @return An error object pointer, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::HaltMove( void )
{
   SetControlWord( 0x010F );
   return 0;
}

/***************************************************************************/
/**
  Set the linkage control word.

  @param value The control word value to set
  @return A pointer to an error object, or NULL on success
  */
/***************************************************************************/
const Error *Linkage::SetControlWord( uint16 value )
{
   cml.Debug( "Link %d control 0x%04x\n", amp[0]->GetNodeID(), value );

   const Error *err = ctrlPDO.Transmit( value );

   if( err ) return err;

   for( int i=0; i<ampct; i++ )
      amp[i]->lastCtrlWord = value;

   return 0;
}

/***************************************************************************/
/**
  Wait for a linkage event condition. This function can be used to wait
  on any generic event associated with the linkage.
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever.
  @param match Returns the matching event condition.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitEvent( Event &e, int32 timeout, LINK_EVENT &match )
{
   const Error *err = e.Wait( eventMap, timeout );
   match = (LINK_EVENT)e.getMask();
   return err;
}

/***************************************************************************/
/**
  Wait for a linkage event condition. This function can be used to wait
  on any generic event associated with the linkage.
  @param e The event to wait on.
  @param timeout The timeout for the wait (milliseconds).  If < 0, then 
  wait forever (default).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitEvent( Event &e, int32 timeout )
{
   LINK_EVENT match;
   return WaitEvent( e, timeout, match );
}

/***************************************************************************/
/**
  Wait for the currently running move to finish, or for an error to occur.

  @param timeout The maximum time to wait (milliseconds).  Default is -1 (forever).
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::WaitMoveDone( int32 timeout )
{
   cml.Debug( "Link %d waiting on move\n", amp[0]->GetNodeID() );

   uint32 value = LINKEVENT_MOVEDONE | LINKEVENT_NODEGUARD | LINKEVENT_FAULT |
		  LINKEVENT_ERROR | LINKEVENT_DISABLED | LINKEVENT_QUICKSTOP | 
	          LINKEVENT_ABORT;

   EventAny e( value );

   LINK_EVENT match;
   const Error *err = WaitEvent( e, timeout, match );

   if( !err )
   {
      match = (LINK_EVENT)(match & value);
      if( match == LINKEVENT_MOVEDONE )
	 return 0;

      // There should be a latched error
      int ndx;
      err = GetLatchedError( ndx );

      // If not, take a best guess
      if( !err ) 
	 err = GetError( match );

      cml.Debug( "Linkage::WaitMoveDone returned: %s\n", err->toString() );
   }

   return err;
}

/***************************************************************************/
/**
  Return an error code for a failed move.  The passed mask identifies
  which amplifier in the linkage generated the error.
  @param mask The event mask that caused the error
  @return A pointer to an error object
  */
/***************************************************************************/
const Error *Linkage::GetError( uint32 mask )
{
   const Error *err, *someErr=0;
   int i, someAmp=-1;


   // Try to find an amplifier with an event mask equal 
   // to the passed mask.
   for( i=0; i<ampct; i++ )
   {
      err = amp[i]->GetErrorStatus();
      if( !err ) continue;

      someErr = err;
      someAmp = i;

      AMP_EVENT ae;
      amp[i]->GetEventMask( ae );

      if( mask == (uint32)ae )
      {
	 LatchError( err, i );
	 return err;
      }
   }

   // If no exact match was found, just return an error 
   // reported by one of the amplifiers.
   if( someAmp >= 0 )
   {
      LatchError( someErr, someAmp );
      return someErr;
   }

   // If all else fails, return a generic error 
   LatchError( &LinkError::UnknownAmpErr, -1 );
   return &LinkError::UnknownAmpErr;
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Get the next PVT segment.  This function is called by an amplifier trajectory
  object when it requires a new trajectory point and doesn't have one cached.
  The linkage trajectory is queried for it's next point, and after this point
  is converted from axis space to amplifier space, the point is distributed to
  all amplifier trajectory objects.

  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Linkage::RequestNextTrjPoint( void )
{
   const Error *err;

   if( !linkTrjPtr )
      return &LinkError::NoActiveTrj;

   uunit pos[ CML_MAX_AMPS_PER_LINK ];
   uunit vel[ CML_MAX_AMPS_PER_LINK ];
   uint8 time;

   bool useVel = linkTrjPtr->UseVelocityInfo();
   err = linkTrjPtr->NextSegment( pos, vel, time );
   if( err ) return err;

   err = ConvertAxisToAmp( pos, vel );
   if( err ) return err;

   for( int i=0; i<ampct; i++ )
      ampTrj[i].AddPoint( pos[i], vel[i], time, useVel );

   return 0;
}

/***************************************************************************/
/**
  Increment a local counter indicating that the linkage trajectory is in use
  by one more axis.
  @return A pointer to an error object, or NULL On success
  */
/***************************************************************************/
const Error *Linkage::IncTrjUseCount( void )
{
   if( linkTrjPtr == 0 )
      return &LinkError::NoActiveTrj;

   if( trjUseCount++ == 0 )
      return linkTrjPtr->StartNew();
   else
      return 0;
}

/***************************************************************************/
/**
  Decrement a local counter indicating the number of axes currently using the
  linkage trajectory.
  @return A pointer to an error object, or NULL On success
  */
/***************************************************************************/
const Error *Linkage::DecTrjUseCount( void )
{
   if( linkTrjPtr == 0 )
      return &LinkError::NoActiveTrj;

   if( --trjUseCount == 0 )
   {
      linkTrjPtr->Finish();
      linkTrjPtr = 0;
   }

   return 0;
}
#endif

/***************************************************************************/
/**
  Update the status event map used by this linkage.  
  */
/***************************************************************************/

#define ERROR_EVENTS       (LINKEVENT_NODEGUARD | LINKEVENT_FAULT | LINKEVENT_ERROR | \
			    LINKEVENT_QUICKSTOP | LINKEVENT_ABORT | LINKEVENT_DISABLED )

void Linkage::UpdateStatus( void )
{
   uint32 orMask = 0;
   uint32 andMask = 0xffffffff;

   uint32 errors = ERROR_EVENTS;

   if( cfg.haltOnPosWarn ) 
      errors |= LINKEVENT_POSWARN;

   if( cfg.haltOnVelWin )
      errors |= LINKEVENT_VELWIN;

   for( int i=0; i<ampct; i++ )
   {
      AMP_EVENT e;

      amp[i]->GetEventMask( e );

      orMask |= (uint32)e;
      andMask &= (uint32)e;

      // If an error condition is being reported, latch it
      if( e & errors )
      {
	 // Check for any real errors
	 const Error *err = amp[i]->GetErrorStatus( true );

	 // If none, it's probably a tracking warning or velocity window
	 if( !err )
	 {
	    if( e & LINKEVENT_POSWARN )
	       err = &AmpError::TrackWarn;
	    else if( e & LINKEVENT_VELWIN )
	       err = &AmpError::VelWin;
	    else
	       err = &AmpError::Unknown;
	 }

	 LatchError( err, i );
	 cml.Warn( "Link %d error latched for amp %d: %s\n", amp[0]->GetNodeID(), i, err->toString() );
      }
   }
   orMask &= ( ERROR_EVENTS | LINKEVENT_POSWARN | 
               LINKEVENT_POSWIN | LINKEVENT_VELWIN | 
               LINKEVENT_POSLIM | LINKEVENT_NEGLIM | 
               LINKEVENT_SOFTLIM_POS | LINKEVENT_SOFTLIM_NEG );

   andMask &= ( LINKEVENT_MOVEDONE | LINKEVENT_TRJDONE );

   orMask |= andMask;

   cml.Debug( "Link %d status: 0x%08x\n", amp[0]->GetNodeID(), orMask );
   eventMap.setMask( orMask );
}

/***************************************************************************/
/**
  Linkage thread.  This thread monitors the linkage during run time.
  */
/***************************************************************************/
void Linkage::run( void )
{
   const Error *err;
   EventAny  allDone( LINKEVENT_MOVEDONE );
   EventAny  doneEvent;
   int errAmp;

   uint32 doneValue(  LINKEVENT_MOVEDONE | LINKEVENT_NODEGUARD | LINKEVENT_FAULT |
		      LINKEVENT_ERROR | LINKEVENT_DISABLED | LINKEVENT_QUICKSTOP | 
		      LINKEVENT_ABORT );
   LINK_EVENT match;


   while( 1 )
   {
      // Wait for a move to start on this linkage.
      err = startSema.Get();

      // This should never fail, but if it does just delay and try again.
      if( err )
      {
	 sleep( 100 );
	 continue;
      }

      // Now, wait for the move to finish or an error to occur.
      cml.Debug( "Link thread waiting for move to finish.\n" );

      // Set or clear the position warning bit depending on whether
      // the linkage is configured to watch it.
      if( cfg.haltOnPosWarn ) 
	 doneValue |= LINKEVENT_POSWARN;
      else 
	 doneValue &= ~LINKEVENT_POSWARN;

      // Same thing for velocity window
      if( cfg.haltOnVelWin )
	 doneValue |= LINKEVENT_VELWIN;
      else
	 doneValue &= ~LINKEVENT_VELWIN;

      // Wait for any of my selected events
      doneEvent.setValue( doneValue );

      err = WaitEvent( doneEvent, -1, match );
      match = (LINK_EVENT)(match & doneValue);

      if( err )
	 cml.Debug( "Link %d error waiting on move done: %s\n",  amp[0]->GetNodeID(), err->toString() );

      else if( match == LINKEVENT_MOVEDONE )
      {
	 cml.Debug( "Link thread done OK\n" );
	 continue;
      }

      else if( (err = GetLatchedError( errAmp )) == 0 )
      {
	 cml.Debug( "Link %d stopped move with unexpected event: 0x%08x\n", 
	            amp[0]->GetNodeID(), match );
      }

      else
	 cml.Debug( "Link %d error from amp %d while waiting on move.\n  %s\n",  
	            amp[0]->GetNodeID(), errAmp, err->toString() );

      // On error, halt the linkage
      HaltMove();
      allDone.Wait( eventMap, -1 );
   }
}

/***************************************************************************/
/**
  When the status of an amplifier connected to this linkage is updated, this
  function is called.  It simply causes the linkage status to be updated also.
  @return false
  */
/***************************************************************************/
bool Linkage::StateEvent::isTrue( uint32 mask )
{
   link->UpdateStatus();
   return false;
}

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
/***************************************************************************/
/**
  Initialize the amplifier trajectory structure.  This is called from the 
  Linkage object constructor.
  @param lptr Points to the linkage that owns this object.
  */
/***************************************************************************/
void Linkage::AmpTrj::Init( Linkage *lptr )
{
   linkPtr = lptr;
   head = tail = 0;
   inUse = false;
}

/***************************************************************************/
/**

*/
/***************************************************************************/
const Error *Linkage::AmpTrj::StartNew( void )
{
   if( inUse ) 
      return &LinkError::AmpTrjInUse;

   inUse = true;
   return linkPtr->IncTrjUseCount();
}

/***************************************************************************/
/**

*/
/***************************************************************************/
void Linkage::AmpTrj::Finish( void )
{
   inUse = false;
   head = tail = 0;
   linkPtr->DecTrjUseCount();
}

/***************************************************************************/
/**

*/
/***************************************************************************/
const Error *Linkage::AmpTrj::AddPoint( uunit pos, uunit vel, uint8 time, bool useVel )
{
   int newHead = (head+1) % CML_LINKAGE_TRJ_BUFFER_SIZE;

   if( newHead == tail )
      return &LinkError::AmpTrjOverflow;

   p[head] = pos;
   v[head] = vel;
   t[head] = time;
   u[head] = useVel;

   head = newHead;

   return 0;
}

/***************************************************************************/
/**
   Get info about velocity usage from the linkage trajectory.
  */
/***************************************************************************/
bool Linkage::AmpTrj::UseVelocityInfo( void )
{
   if( head == tail )
      linkPtr->RequestNextTrjPoint();

   return u[tail];
}

/***************************************************************************/
/**
  Get the buffer size.  This just uses the buffer size info from the 
  owned Linkage trajectory
  */
/***************************************************************************/
int Linkage::AmpTrj::MaximumBufferPointsToUse( void )
{
   if( !linkPtr->linkTrjPtr )
      return Trajectory::MaximumBufferPointsToUse();
   return linkPtr->linkTrjPtr->MaximumBufferPointsToUse();
}

/***************************************************************************/
/**
  Get the next segment for the PVT move.
  */
/***************************************************************************/
const Error *Linkage::AmpTrj::NextSegment( uunit &pos, uunit &vel, uint8 &time )
{
   if( !inUse ) return &LinkError::AmpTrjNotRunning;

   const Error *err;

   if( head == tail )
   {
      err = linkPtr->RequestNextTrjPoint();
      if( err ) return err;
   }

   pos  = p[tail];
   vel  = v[tail];
   time = t[tail];

   tail = (tail+1) % CML_LINKAGE_TRJ_BUFFER_SIZE;

   return 0;
}
#endif

/***************************************************************************/
/**
  Latch an error if one isn't already being held.
  @param err Points to the error
  @param ndx Index of the amp that caused it, or -1 if not known.
  @return The new latched error.
  */
/***************************************************************************/
const Error *Linkage::LatchError( const Error *err, int ndx )
{
   if( !latchedErr )
   {
      latchedErr = err;
      latchedErrAmp = ndx;
   }
   return latchedErr;
}

/***************************************************************************/
/**
  Initialize the receive PDO used to control words to each amplifier
  held by a linkage.  The COB ID used for this PDO is the standard
  ID used for RPDO 1 of the first axis.

  @return An error object pointer on failure, NULL on success
  */
/***************************************************************************/
const Error *RPDO_LinkCtrl::Init( void )
{
   int32 cobID = link[0].GetRpdoCobID( (uint16)1 );

   const Error *err = RPDO::Init( cobID );

   if( !err ) err = ctrl.Init( OBJID_CONTROL );
   if( !err ) err = AddVar( ctrl );
   if( !err ) err = SetType( 255 );
   if( err ) return err;

   for( int i=0; i<link.GetAmpCount(); i++ )
   {
      err = link[i].PdoSet( 1, *this );
      if( err ) return err;
   }

   return 0;
}

/***************************************************************************/
/**
  Transmit a control word using this PDO.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *RPDO_LinkCtrl::Transmit( uint16 c )
{
   ctrl.Write( c );
   return RPDO::Transmit( link[0].GetCanOpen() );
}

/***************************************************************************/
/**
  Default constructor.  All settings are set to their default values at
  construction time.
  */
/***************************************************************************/
LinkSettings::LinkSettings()
{
   moveAckTimeout = 200;
   haltOnPosWarn = false;
   haltOnVelWin = false;
}

