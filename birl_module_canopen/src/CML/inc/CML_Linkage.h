/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the Linkage object.

A linkage is a group of two or more amplifiers which
work together.

*/

#ifndef _DEF_INC_LINKAGE
#define _DEF_INC_LINKAGE

#include "CML_Settings.h"
#include "CML_Amp.h"
#include "CML_EventMap.h"
#include "CML_Geometry.h"
#include "CML_TrjScurve.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
Linkage events.  This enumeration provides a list of events that can be
used to wait on linkage conditions.

In general, linkage events parallel the amplifier events of all of the 
amplifiers attached to the linkage.  For example, if any of the amplifiers
is reporting an error event, then the linkage will be reporting an error
event.
*/
/***************************************************************************/
enum LINK_EVENT
{
   /// Set when all amplifiers attached to this linkage have finished their moves and
   /// have settled in to position at the end of the move.  Cleared when a new move is 
   /// started on any amplifier.
   LINKEVENT_MOVEDONE      = 0x00000001,

   /// Set when all amplifiers attached to the linkage have finished their moves, but
   /// have not yet settled into position at the end of the move.  Cleared when a new 
   /// move is on any amplifier started.
   LINKEVENT_TRJDONE       = 0x00000002,

   /// A node guarding (or heartbeat) error has occurred.  This indicates that
   /// one of the amplifiers failed to respond within the expected amount of 
   /// time for either a heartbeat or node guarding message.
   LINKEVENT_NODEGUARD     = 0x00000004,

   /// A latching fault has occurred on one of the amplifiers attached to this linkage.
   LINKEVENT_FAULT         = 0x00000010,

   /// A non-latching error has occurred on one of the amplifiers.  
   LINKEVENT_ERROR         = 0x00000020,

   /// One of the the amplifiers is reporting a position warning event.
   LINKEVENT_POSWARN       = 0x00000040,

   /// One of the amplifiers is reporting a position window event.
   LINKEVENT_POSWIN        = 0x00000080,

   /// One of the amplifiers is reporting a velocity window event.
   LINKEVENT_VELWIN        = 0x00000100,

   /// One of the amplifiers is currently disabled.
   LINKEVENT_DISABLED      = 0x00000200,

   /// The positive limit switch of one or more amplifier is currently active
   LINKEVENT_POSLIM        = 0x00000400,

   /// The negative limit switch of one or more amplifier is currently active
   LINKEVENT_NEGLIM        = 0x00000800,

   /// The positive software limit of one or more amplifier is currently active
   LINKEVENT_SOFTLIM_POS   = 0x00001000,

   /// The negative software limit of one or more amplifier is currently active
   LINKEVENT_SOFTLIM_NEG   = 0x00002000,

   /// One of the linkage amplifiers is presently performing a quick stop sequence
   /// or is holding in quick stop mode.  The amplifier must be disabled to clear this.
   LINKEVENT_QUICKSTOP     = 0x00004000,

   /// One or more amplifier aborted the last profile without finishing.
   LINKEVENT_ABORT         = 0x00008000
};

/***************************************************************************/
/**
This class represents error conditions that can occur in the Linkage class.
*/
/***************************************************************************/
class LinkError: public Error
{
public:
   /// The amplifier objects used to init the linkage are
   /// not all attached to the same CanOpen network.
   static const LinkError NetworkMismatch;

   /// An illegal number of amplifiers was passed 
   /// to Linkage::Init
   static const LinkError BadAmpCount;

   /// Init was called on a Linkage that is already initialized.
   static const LinkError AlreadyInit;

   /// The passed amplifier object is already assigned to a linkage
   static const LinkError AmpAlreadyLinked;

   /// The point dimension doesn't match the number of linkage axes
   static const LinkError AxisCount;

   /// Amplifier trajectory structure overflow
   static const LinkError AmpTrjOverflow;  

   /// Amplifier trajectory already in use
   static const LinkError AmpTrjInUse;     

   /// Amplifier trajectory not presently in use
   static const LinkError AmpTrjNotRunning;

   /// No linkage trajectory is active
   static const LinkError NoActiveTrj;

   /// A zero or negative move limit was detected
   static const LinkError BadMoveLimit;

   /// Unknown amplifier error 
   static const LinkError UnknownAmpErr;

   /// Timeout waiting on amplifier to respond to start move command
   static const LinkError StartMoveTO;

   /// Returned if Linkage::MoveTo is called on a system where floating
   /// point math was not enabled at compile time.
   static const LinkError NotSupported;

protected:
   /// Standard protected constructor
   LinkError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**
Receive PDO used to update the control word of all amplifiers in the linkage.
This object is intended for internal use only.
*/
/***************************************************************************/
class RPDO_LinkCtrl: public RPDO
{
   /// Points to the Linkage that owns this PDO
   class Linkage &link;
   Pmap16 ctrl;

   /// Private copy constructor (not supported)
   RPDO_LinkCtrl( const RPDO_LinkCtrl & );

   /// Private assignment operator (not supported)
   RPDO_LinkCtrl &operator=( const RPDO_LinkCtrl& );

public:
   /// Default constructor for this PDO
   RPDO_LinkCtrl( class Linkage &l ): link(l) {}
   const Error *Init( void );
   const Error *Transmit( uint16 c );
};

/***************************************************************************/
/**
Linkage object settings.  An object of this type may be passed to the
Linkage::Configure function to define the settings used by that linkage.
*/
/***************************************************************************/
class LinkSettings
{
public:
   LinkSettings();

   /// This setting gives the amount of time (milliseconds)
   /// to wait for all amplifiers to acknowedge the start
   /// of a new move before reporting an error.
   ///
   /// When a new move is started on the linkage, each 
   /// amplifier will respond with an acknowledgment.
   /// If all of these responses are not received in 
   /// this amount of time then an error will be 
   /// reported.
   ///
   /// Default: 200 ms
   int32 moveAckTimeout;

   /// If this setting is set to true, then the linkage object
   /// will automatically issue a halt to all axes if any of
   /// them reports a position warning window condition during
   /// a move.
   ///
   /// Default: false
   bool haltOnPosWarn;

   /// If this setting is set to true, then the linkage object
   /// will automatically issue a halt to all axes if any of
   /// them reports a velocity tracking window condition during
   /// a move.
   ///
   /// Default: false
   bool haltOnVelWin;
};

/***************************************************************************/
/**
Linkage object, used for controlling a group of coordinated amplifiers.
*/
/***************************************************************************/
class Linkage: public Thread
{
   /// Private copy constructor (not supported)
   Linkage( const Linkage & );

   /// Private assignment operator (not supported)
   Linkage &operator=( const Linkage & );

public:
   Linkage();
   virtual ~Linkage();

   const Error *Init( uint16 ct, Amp a[] );
   const Error *Init( uint16 ct, Amp *a[] );
   const Error *Configure( LinkSettings &settings );

   Amp &GetAmp( uint16 i );

   const Error *MoveTo( PointN &p, uunit vel, uunit acc, uunit dec, uunit jrk, bool start=true );
   const Error *SetMoveLimits( uunit vel, uunit acc, uunit dec, uunit jrk );
   const Error *GetMoveLimits( uunit &vel, uunit &acc, uunit &dec, uunit &jrk );
   const Error *MoveTo( PointN &p, bool start=true );
   const Error *StartMove( void );
   const Error *WaitMoveDone( int32 timeout=-1 );
   const Error *WaitEvent( Event &e, int32 timeout, LINK_EVENT &match );
   const Error *WaitEvent( Event &e, int32 timeout=-1 );
   const Error *HaltMove( void );

   const Error *SendTrajectory( LinkTrajectory &trj, bool start=true );

   /// Return any latched error codes held by the linkage object.
   /// When an error occurs during a move, the linkage latches the first
   /// error to occur and the index of the amplifier that caused it.
   ///
   /// Note that the latched error information will be reset automatically
   /// at the start of any new move. 
   /// 
   /// @param amp The index of the amplifier producing the latched error
   /// will be returned.  -1 will be returned if the amplifier is unknown.
   /// @return A pointer to the latched error object, or NULL if no error
   /// was latched.
   const Error *GetLatchedError( int &amp )
   {
      amp = latchedErrAmp;
      return latchedErr;
   }

   /// Clear any latched errors.  This function clears the latched error
   /// information returned by Linkage::GetLatchedError().
   /// Latched errors are automatically cleared at the start of a new move.  
   /// This call may be used to clear latched error information at any other time.
   void ClearLatchedError( void )
   {
      latchedErrAmp = -1;
      latchedErr = 0;
   }

   /// Return a reference to the specified amplifier object 
   /// in this linkage.  This is the same as Linkage::GetAmp
   /// @param i The amplifier index location
   /// @return A reference to the amp object
   Amp &operator[]( uint16 i )
   {
      return GetAmp(i);
   }

   /// Return the number of amplifiers associated with this linkage
   /// @return The amplifier count.
   uint16 GetAmpCount( void )
   {
      return ampct;
   }

   /// Return the number of independent axes associated with this linkage.
   /// For a standard Linkage object, this will be the same as the amp count,
   /// however, this function is virtual to allow more complex structures to
   /// be represented in sub-classes.
   /// @return The number of independent axes for this Linkage.
   virtual uint16 GetAxesCount( void )
   {
      return ampct;
   }

   const Error *GetPositionCommand( PointN &p );

   /// Convert the linkage position from the amplifier frame to the axis frame.
   /// The passed array contains a position for each amplifer on entry.  These
   /// positions should be converted to axis positions in this function.  By 
   /// default, this function doesn't do anything, however it is a virtual 
   /// function to allow it to be extended in sub-classes.
   /// @param pos An array of amplifer positions on entry, and axes positions
   ///            on exit.
   /// @return NULL on success or an Error pointer on failure.
   virtual const Error *ConvertAmpToAxisPos( uunit pos[] ){ return 0; }

   /// Convert the linkage position from the axis frame to the amplifier frame.
   /// The passed array contains a position for each axis on entry.  These
   /// positions should be converted to amplifier positions in this function.  By 
   /// default, this function doesn't do anything, however it is a virtual 
   /// function to allow it to be extended in sub-classes.
   /// @param pos An array of axis positions on entry, and amp positions
   ///            on exit.
   /// @return NULL on success or an Error pointer on failure.
   virtual const Error *ConvertAxisToAmpPos( uunit pos[] ){ return 0; }

   /// Convert position & velocity information from the amplifier frame to the
   /// axis frame.
   ///
   /// The passed arrays contain a position and velocity for each amplifer on entry.
   /// These values should be converted to axis positions & velocities in this function.
   ///
   /// By default, this function doesn't do anything, however it is a virtual 
   /// function to allow it to be extended in sub-classes.
   /// @param pos An array of amplifer positions on entry, and axes positions
   ///            on exit.
   /// @param vel An array of amplifier velocities on entry, and axis velocities
   ///            on exit.
   /// @return NULL on success or an Error pointer on failure.
   virtual const Error *ConvertAmpToAxis( uunit pos[], uunit vel[] ){ return 0; }

   /// Convert position & velocity information from the axis frame to the amplifier frame.
   ///
   /// The passed arrays contain a position and velocity for each axis on entry.
   /// These values should be converted to amp positions & velocities in this function.
   ///
   /// By default, this function doesn't do anything, however it is a virtual 
   /// function to allow it to be extended in sub-classes.
   /// @param pos An array of axis positions on entry, and amplifier positions
   ///            on exit.
   /// @param vel An array of axis velocities on entry, and amplifier velocities
   ///            on exit.
   /// @return NULL on success or an Error pointer on failure.
   virtual const Error *ConvertAxisToAmp( uunit pos[], uunit vel[] ){ return 0; }

   /// This is an event map that is used to track linkage
   /// events and state changes
   EventMap eventMap;

private:
   LinkSettings cfg;
   RPDO_LinkCtrl ctrlPDO;
   uint16 ampct;
   Amp *amp[ CML_MAX_AMPS_PER_LINK ];
   uunit maxVel, maxAcc, maxDec, maxJrk;
   Semaphore startSema;

#ifdef CML_ALLOW_FLOATING_POINT
   LinkTrjScurve scurve;
#endif

   int latchedErrAmp;
   const Error *latchedErr;

   const Error *LatchError( const Error *err, int ndx );

   void CheckIndex( uint16 i );

#ifdef CML_LINKAGE_TRJ_BUFFER_SIZE
   /// Utility class used internally by the linkage object
   class AmpTrj: public Trajectory
   {
      Linkage *linkPtr;
      uint8 ampNum, head, tail;
      bool inUse;
      uunit p[ CML_LINKAGE_TRJ_BUFFER_SIZE ];
      uunit v[ CML_LINKAGE_TRJ_BUFFER_SIZE ];
      uint8 t[ CML_LINKAGE_TRJ_BUFFER_SIZE ];
      bool  u[ CML_LINKAGE_TRJ_BUFFER_SIZE ];

   public:
      void Init( Linkage *lptr );
      const Error *StartNew( void );
      void Finish( void );
      bool UseVelocityInfo( void );
      int MaximumBufferPointsToUse( void );
      const Error *NextSegment( uunit &pos, uunit &vel, uint8 &time );
      const Error *AddPoint( uunit pos, uunit vel, uint8 time, bool useVel );
   };
   friend class AmpTrj;

   const Error *RequestNextTrjPoint( void );

   LinkTrajectory *linkTrjPtr;
   AmpTrj ampTrj[ CML_MAX_AMPS_PER_LINK ];
#endif

   /// Utility class used to keep the linkage status up to date
   /// as the status of it's amplifiers change.
   class StateEvent: public Event
   {
      public:
	 Linkage *link;
	 StateEvent( void ): Event(0){}
	 bool isTrue( uint32 mask );
   };
   friend class StateEvent;
   StateEvent stateEvent[ CML_MAX_AMPS_PER_LINK ];

   int trjUseCount;
   const Error *IncTrjUseCount( void );
   const Error *DecTrjUseCount( void );
   const Error *SetControlWord( uint16 value );
   const Error *GetError( uint32 mask );
   void run( void );

   void UpdateStatus( void );

   void InvalidateAmp( Amp *a );
   friend class Amp;

// add 2011.1.10
public:
   /// Çå³ý¹ì¼£´íÎó
   void ClearTrjError( void )
   {	   
	   int i;
	   linkTrjPtr = 0;
	   for(i = 0; i < CML_MAX_AMPS_PER_LINK; i++)
	   {
		   ampTrj[i].Finish();
	   }   
   }
// end of add
};

CML_NAMESPACE_END()
#endif

