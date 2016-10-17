/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the Copley Amplifier object.  This object may be
used to interface to an amplifier over the CANopen network.

*/

#ifndef _DEF_INC_COPLEYAMP
#define _DEF_INC_COPLEYAMP

#include "CML_Settings.h"
#include "CML_AmpDef.h"
#include "CML_AmpStruct.h"
#include "CML_CanOpen.h"
#include "CML_EventMap.h"
#include "CML_Copley.h"
#include "CML_PDO.h"
#include "CML_Trajectory.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This class represents error conditions that can occur in the Copley Amplifier
object.
*/
/***************************************************************************/
class AmpError: public NodeError
{
public:
   static const AmpError Fault;         ///< Latching fault is active
   static const AmpError ShortCircuit;  ///< Short circuit detected
   static const AmpError AmpTemp;       ///< Amplifier over temp.
   static const AmpError OverVolt;      ///< Amplifier over voltage
   static const AmpError UnderVolt;     ///< Amplifier under voltage
   static const AmpError MotorTemp;     ///< Motor over temp.
   static const AmpError EncoderPower;  ///< Encoder power error
   static const AmpError PhaseErr;      ///< Motor phasing error
   static const AmpError TrackErr;      ///< Position tracking error
   static const AmpError NodeState;     ///< The drive's state is inappropriate for the requested operation
   static const AmpError pvtSegPos;     ///< PVT segment position value over 24-bits
   static const AmpError pvtSegVel;     ///< PVT segment velocity value too large        
   static const AmpError pvtBufferFull; ///< PVT trajectory buffer full
   static const AmpError badDeviceID;   ///< Unknown device identity
   static const AmpError badHomeParam;  ///< Bad parameter specified to home command
   static const AmpError badMoveParam;  ///< Bad parameter specified to move command
   static const AmpError InMotion;      ///< Amplifier is currently in motion
   static const AmpError GuardError;    ///< The amplifier's heartbeat message timed out
   static const AmpError PosLim;        ///< Positive limit switch is active
   static const AmpError NegLim;        ///< Negative limit switch is active
   static const AmpError PosSoftLim;    ///< Positive software limit is active
   static const AmpError NegSoftLim;    ///< Negative software limit is active
   static const AmpError TrackWarn;     ///< Position tracking warning
   static const AmpError Unknown;       ///< An error occurred, but went away before it could be decoded.
   static const AmpError Reset;         ///< The amplifier has been reset.
   static const AmpError Disabled;      ///< The amplifier is disabled
   static const AmpError QuickStopMode; ///< The amplifier is doing a quick stop
   static const AmpError NoUserUnits;   ///< User units are not available (see CML_Settings.h)
   static const AmpError Abort;         ///< Last trajectory aborted
   static const AmpError pvtPosUnavail; ///< The PVT segment position is not available.
   static const AmpError VelWin;        ///< Velocity tracking window exceeded

   static const AmpError *DecodeStatus( EVENT_STATUS stat );
protected:
   /// Standard protected constructor
   AmpError( uint16 id, const char *desc ): NodeError( id, desc ){}
};

/***************************************************************************/
/**
This class represents latching amplifier fault conditions.  
*/
/***************************************************************************/
class AmpFault: public AmpError
{
public:
   static const AmpFault Memory;         ///< Fatal hardware error: the flash data is corrupt.         
   static const AmpFault ADC;            ///< Fatal hardware error: An A/D offset error has occurred.  
   static const AmpFault ShortCircuit;   ///< The amplifier detected a short circuit condition         
   static const AmpFault AmpTemp;        ///< The amplifier is over temperature                        
   static const AmpFault MotorTemp;      ///< A motor temperature error was detected                   
   static const AmpFault OverVolt;       ///< The amplifier bus voltage is over the acceptable limit   
   static const AmpFault UnderVolt;      ///< The amplifier bus voltage is below the acceptable limit  
   static const AmpFault EncoderPower;   ///< Over current on the encoder power supply                 
   static const AmpFault PhaseErr;       ///< Amplifier phasing error                                  
   static const AmpFault TrackErr;       ///< Tracking error, the position error is too large.         
   static const AmpFault I2TLimit;       ///< Current limited by I^2 algorithm.         
   static const AmpFault Unknown;		  ///< Some unknown amplifier fault has occurred

   static const AmpFault *DecodeFault( AMP_FAULT f );
protected:
   /// Standard protected constructor
   AmpFault( uint16 id, const char *desc ): AmpError( id, desc ){}
};

/***************************************************************************/
/**
Transmit PDO used to send out status word updates.
*/
/***************************************************************************/
class TPDO_Status: public TPDO
{
   class Amp *ampPtr;                ///< Points to the amplifier to which this PDO belongs
public:
   Pmap16 status;                    ///< Used to map the CAN status word passed in the PDO
   Pmap32 estat;                     ///< Used to map the 'event status' word to the PDO
   Pmap16 inputs;

   /// Default constructor for this PDO
   TPDO_Status(){}
   const Error *Init( Amp &amp, uint16 slot );
   virtual void Received( void );
private:
   void UpdateEvents( uint16 stat, uint32 events, uint16 inputs );

   /// Private copy constructor (not supported)
   TPDO_Status( const TPDO_Status& );

   /// Private assignment operator (not supported)
   TPDO_Status& operator=( const TPDO_Status& );

};

/***************************************************************************/
/**
Transmit PDO used to send out PVT buffer status updates
*/
/***************************************************************************/
class TPDO_PvtBuffStat: public TPDO
{
   class Amp *ampPtr;                ///< Points to the amplifier to which this PDO belongs
   Pmap32 stat;                      ///< Used to map the PVT buffer status information
   uint8 ct;

public:
   /// Default constructor for this PDO
   TPDO_PvtBuffStat(){}
   const Error *Init( Amp &amp, uint16 slot );
   virtual void Received( void );

private:
   /// Private copy constructor (not supported)
   TPDO_PvtBuffStat( const TPDO_PvtBuffStat& );

   /// Private assignment operator (not supported)
   TPDO_PvtBuffStat& operator=( const TPDO_PvtBuffStat& );
};

/***************************************************************************/
/**
Receive PDO used to send PVT segments to the amplifier
*/
/***************************************************************************/
class RPDO_Pvt: public RPDO
{
   class Amp *ampPtr;                ///< Points to the amplifier to which this PDO belongs
   PmapRaw dat;                      ///< Used to map PVT buffer information passed to the amplifier

public:
   /// Default constructor for this PDO
   RPDO_Pvt(){}

   const Error *Init( Amp &amp, uint16 slot );

   const Error *Transmit( byte *data );
private:
   /// Private copy constructor (not supported)
   RPDO_Pvt( const RPDO_Pvt& );

   /// Private assignment operator (not supported)
   RPDO_Pvt& operator=( const RPDO_Pvt& );
};

/***************************************************************************/
/**
Copley amplifier settings object.  This object is passed to the Init() 
method of the Copley amp.  It holds the various customizable settings 
used by the amplifier.  
*/
/***************************************************************************/
class AmpSettings
{
public:
   /// Create a settings object with all default values.
   /// The default values for each member of this class are defined below.
   AmpSettings();

   /// Synch object period in microseconds.
   /// The synch object is a message that is transmitted by one node on a 
   /// CANopen network at a fixed interval.  This message is used to 
   /// synchronize the devices on the network.
   ///
   /// Default is 10,000 (10 ms).
   uint32 synchPeriod;

   /// Synch object CAN message ID.
   /// This is the message ID used for the synch message.
   /// Default is 0x00000080
   uint32 synchID;

   /// Use first initialized amplifier as synch producer.
   /// If this setting is true (default), then the first amplifier
   /// to be initialized will be set as the synch producer, and
   /// all other amplifier's will be setup as synch consumers.
   /// This causes the value of the 'synchProducer' setting
   /// to be updated during init to indicate whether the amp
   /// is producing synch messages or not.
   ///
   /// By default, this setting is true
   bool synchUseFirstAmp;

   /// Synch producer (true/false)
   /// If true, this node will produce synch messages.
   /// Default: false
   ///
   /// Note: If the 'synchUseFirstAmp' setting of this object is true,
   ///       then the passed value of this settings will not be used, 
   ///       and will be overwritten during initialization.
   ///
   /// Note: There should be exactly one synch producer on every network.
   bool synchProducer;

   /// High resolution time stamp CAN ID.  The high resolution time stamp
   /// is a PDO that is generated by the synch producer and consumed by 
   /// the other amplifiers on the network.  It is used to synchronize the
   /// clocks of the amplifiers.  This parameter defines the CAN ID that
   /// will be used for this message.
   /// Setting to zero will disable the time stamp message.
   /// Default 0x0180
   uint32 timeStampID;

   /// The CANopen heartbeat protocol is one of two standard methods used
   /// to constantly watch for network or device problems.  
   /// When the heartbeat protocol is used, each device on the CANopen
   /// network transmits a 'heartbeat' message at a specified interval.
   /// The network master watches for these messages, and is able to 
   /// detect a device error if it's heartbeat message is not received
   /// within the expected time.
   ///
   /// This parameter configures the heartbeat period (milliseconds)
   /// that will be used by this amplifier to transmit it's heartbeat
   /// message.
   ///
   /// If this parameter is set to zero, then the heartbeat protocol
   /// is disabled on this node.
   ///
   /// Default: zero (not used)
   uint16 heartbeatPeriod;

   /// Additional time to wait before generating a heartbeat error (milliseconds)
   /// If the heartbeat protocol is used, then this value, combined with the
   /// heartbeatTime will determine how long the network master waits for the
   /// node's heartbeat message before it generates a heartbeat error.
   ///
   /// Note that setting this to zero does not disable the heartbeat protocol.
   /// set the heartbeatPeriod value to zero to disable heartbeat.
   ///
   /// Default 200 (ms)
   uint16 heartbeatTimeout;

   /// Node guarding guard time (milliseconds)
   ///
   /// The CANopen node guarding protocol is a second method (the first being the
   /// heartbeat protocol) for devices on the network to watch for network problems.
   /// In this protocol, the master controller sends a request message out to the 
   /// slave device at a specified interval.  The slave device responds to this 
   /// request with a message indicating it's state.
   ///
   /// The main difference between this protocol and the heartbeat protocol is that
   /// both the slave node and the master are able to recognize network errors.
   /// With the heartbeat protocol only the network master is able to identify 
   /// network problems.
   ///
   /// Note that only one of these two protocols can be active in a node device at
   /// any time.  If the heartbeat period is non-zero, then the heartbeat protocol
   /// will be used.
   ///
   /// This parameter gives the node guarding period for use with this node.  This
   /// is the period between node guarding request messages sent by the master 
   /// controller.
   ///
   /// Note that both this parameter, and the life time factor must be non-zero for
   /// node guarding to be used.
   ///
   /// Default 200 (ms)
   uint16 guardTime;

   /// Node guarding life time factor
   ///
   /// When the node guarding protocol is used, this parameter is used by the slave
   /// device to determine how long to wait for a node guarding request from the 
   /// master controller before signaling an error condition.
   ///
   /// The life time factor is treated as a multiple of the guard time.  
   ///
   /// If this parameter and the node guard time are both non-zero,
   /// and the heartbeatTime is zero, then node guarding will be setup 
   /// for the amplifier.
   ///
   /// Default 3 (multiples of the guard time)
   uint8 lifeFactor;

   /// Enable amplifier at init time.  If this is true, then the amplifier
   /// will be enabled at the end of a successful init().  If false, the 
   /// amplifier will be disabled when init() returns.
   ///
   /// Default: true
   bool enableOnInit;

   /// Initial mode of operation.  This defines the mode of operation that the
   /// amplifier will be placed in when it is initialized.
   ///
   /// Default: AMPMODE_CAN_HOMING
   AMP_MODE initialMode;

   /// Reset the amplifier on init.  If true, the amplifier will be reset 
   /// when it is initialized.  This has the advantage of clearing out any
   /// fault conditions and putting the amplifier in a known state.
   ///
   /// Default: false
   bool resetOnInit;

   /// Max PVT segments to send in response to a PVT status update.  This
   /// parameter may be used to limit the number of new PVT segments to send
   /// in response to a PVT status update.  Normally, this parameter may be
   /// safely left at it's default setting.
   ///
   /// Default 6
   uint8 maxPvtSendCt;
};

/***************************************************************************/
/**
PVT trajectory segment cache object.  This is used internally by the Amp
object to keep track of PVT segments recently sent.  It allows the amp object
to recover if a segment is lost in transit by resending the missing segments.
*/
/***************************************************************************/
class PvtSegCache
{
   #define PVTCACHESIZE  32
public:
   /// Default constructor.  Clears the cache.
   PvtSegCache(){ Clear(); }

   /// Clear the cache
   void Clear(){ ct = top = 0; }

   void AddSegment( uint8 *seg, uint16 id, uunit p );
   bool GetSegment( uint8 *seg, uint16 id );
   bool GetPosition( uunit *p, uint16 id );

private:
   /// Total number of segments in the cache
   uint16 ct;
   /// Index of the oldest segment in the cache
   uint16 top;
   /// ID of the oldest segment in the cache
   uint16 oldest;
   /// Holds copies of the cached segments
   uint8 cache[ PVTCACHESIZE ][8];
   /// Holds position info for each segment.
   uunit pos[ PVTCACHESIZE ];

   /// Private copy constructor (not supported)
   PvtSegCache( const PvtSegCache& );

   /// Private assignment operator (not supported)
   PvtSegCache& operator=( const PvtSegCache& );
};

/***************************************************************************/
/**
Trapezoidal profile parameters.  This structure holds all the parameters 
necessary to perform a trapezoidal profile move.
*/
/***************************************************************************/
struct ProfileConfigTrap
{
   /// For absolute moves this is an absolute position,
   /// for relative moves it's a distance to move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit pos;

   /// Velocity limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit vel;

   /// Acceleration limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit acc;

   /// Deceleration limit for move.
   ///
   /// Note that if this parameter is not set, then the 
   /// acceleration value will be used for deceleration.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit dec;

   /// Default constructor.  Simply set all parameters to zero.
   ProfileConfigTrap( void )
   {
      pos = vel = acc = dec = 0;
   }
};

/***************************************************************************/
/**
S-curve profile parameters.  This structure holds all the parameters 
necessary to perform a s-curve (jerk limited) profile move.
*/
/***************************************************************************/
struct ProfileConfigScurve
{
   /// For absolute moves this is an absolute position,
   /// for relative moves it's a distance to move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit pos;

   /// Velocity limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit vel;

   /// Acceleration limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit acc;

   /// Jerk limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit jrk;

   /// Default constructor.  Simply set all parameters to zero.
   ProfileConfigScurve( void )
   {
      pos = vel = acc = jrk = 0;
   }
};

/***************************************************************************/
/**
Velocity profile parameters.  This structure holds all the parameters 
necessary to perform a velocity profile move.
*/
/***************************************************************************/
struct ProfileConfigVel
{
   /// Direction of motion.  If >= 0, then move in the positive direction
   /// If < 0, then move in the negative direction.
   uunit dir;

   /// Velocity limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit vel;

   /// Acceleration limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit acc;

   /// Deceleration limit for move.
   ///
   /// This parameter is specified in "user units".  See 
   /// Amp::SetCountsPerUnit for details.
   uunit dec;

   /// Default constructor.  Simply set all parameters to zero.
   ProfileConfigVel( void )
   {
      dir = vel = acc = dec = 0;
   }
};

/***************************************************************************/
/**
Copley Controls amplifier object.

This object represents a Copley Controls amplifier on the CANopen network.

The Amp object can be used directly for fairly easy control of an amplifier
on the CANopen network.  The object provides easy to use methods for setting
and getting amplifier parameter blocks.  It also handles many of the details
of both point to point moves and the transfer of complex PVT profiles.

In addition, the standard Amp object provides several virtual functions 
which can be used in derived classes to signal the derived object on 
state changes or emergency conditions.

*/
/***************************************************************************/
class Amp: public CopleyNode
{
public:
   /***************************************************************************/
   /** \name Amplifier initialization
   */
   /***************************************************************************/
   //@{

   /// Default constructor.  Init() must be called before 
   /// the Amp object may be used.
   Amp(){ myLink = 0; }
   Amp( CanOpen &co, int16 nodeID );
   Amp( CanOpen &co, int16 nodeID, AmpSettings &settings );
   virtual ~Amp();

   const Error *Init( CanOpen &co, int16 nodeID );
   const Error *Init( CanOpen &co, int16 nodeID, AmpSettings &settings );
   const Error *ReInit( void );
   const Error *Reset( void );
   //@}

   /***************************************************************************/
   /** \name Amplifier modes & status info
   */
   /***************************************************************************/
   //@{
   const Error *SetAmpMode( AMP_MODE mode );
   const Error *GetAmpMode( AMP_MODE &mode );
   const Error *Disable( bool wait=true );
   const Error *Enable( bool wait=true );
   bool IsHardwareEnabled( void );
   bool IsSoftwareEnabled( void );
   bool IsReferenced( void );
   const Error *CheckStateForMove( void );

   const Error *ClearFaults( AMP_FAULT mask = (AMP_FAULT)-1 );
   const Error *GetFaults( AMP_FAULT &value );
   const Error *SetFaultMask( AMP_FAULT &value );
   const Error *GetFaultMask( AMP_FAULT &value );
   const Error *GetEventStatus( EVENT_STATUS &stat );
   const Error *GetEventSticky( EVENT_STATUS &stat );
   const Error *GetEventLatch( EVENT_STATUS &stat );
   const Error *SetPwmMode( AMP_PWM_MODE mode );
   const Error *GetPwmMode( AMP_PWM_MODE &mode );
   const Error *SetPhaseMode( AMP_PHASE_MODE mode );
   const Error *GetPhaseMode( AMP_PHASE_MODE &mode );
   //@}

   /***************************************************************************/
   /** \name Motor & Amplifier state information
   */
   /***************************************************************************/
   //@{
   const Error *GetPositionActual( uunit &value );
   const Error *SetPositionActual( uunit value );
   const Error *GetPositionMotor( uunit &value );
   const Error *SetPositionMotor( uunit value );
   const Error *GetPositionCommand( uunit &value );
   const Error *GetPositionError( uunit &value );
   const Error *GetVelocityActual( uunit &value );
   const Error *GetVelocityLoad( uunit &value );
   const Error *GetVelocityCommand( uunit &value );
   const Error *GetVelocityLimited( uunit &value );
   const Error *GetCurrentActual( int16 &value );
   const Error *GetCurrentCommand( int16 &value );
   const Error *GetCurrentLimited( int16 &value );
   const Error *GetTrajectoryVel( uunit &value );
   const Error *GetTrajectoryAcc( uunit &value );
   const Error *GetPhaseAngle( int16 &value );
   const Error *GetHallState( int16 &value );
   const Error *GetRefVoltage( int16 &value );
   const Error *GetHighVoltage( int16 &value );
   const Error *GetAmpTemp( int16 &value );
   const Error *GetAnalogEncoder( int16 &sin, int16 &cos );
   const Error *GetMotorCurrent( int16 &u, int16 &v );
   //@}

   /***************************************************************************/
   /** \name Input & Output pin control
   */
   /***************************************************************************/
   //@{
   const Error *SetIoConfig( AmpIoCfg &cfg );
   const Error *GetIoConfig( AmpIoCfg &cfg );
   const Error *GetInputs( uint16 &value, bool viaSDO = false );
   const Error *WaitInputEvent( Event &e, int32 timeout, uint32 &match );
   const Error *WaitInputHigh( uint32 inputs, int32 timeout = -1 );
   const Error *WaitInputLow( uint32 inputs, int32 timeout = -1 );
   const Error *SetIoPullup( uint16 value );
   const Error *GetIoPullup( uint16 &value );
   const Error *SetInputConfig( int8 pin, INPUT_PIN_CONFIG cfg );
   const Error *GetInputConfig( int8 pin, INPUT_PIN_CONFIG &cfg );
   const Error *SetInputDebounce( int8 pin, int16 value );
   const Error *GetInputDebounce( int8 pin, int16 &value );
   const Error *SetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG cfg, uint32 mask1=0, uint32 mask2=0 );
   const Error *GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg );
   const Error *GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg, uint32 &mask );
   const Error *GetOutputConfig( int8 pin, OUTPUT_PIN_CONFIG &cfg, uint32 &mask1, uint32 &mask2 );
   const Error *SetOutputs( uint16 value );
   const Error *GetOutputs( uint16 &value );
   //@}

   /***************************************************************************/
   /** \name Position capture 
   */
   /***************************************************************************/
   //@{
   const Error *SetPosCaptureCfg( POS_CAPTURE_CFG cfg );
   const Error *GetPosCaptureCfg( POS_CAPTURE_CFG &cfg );
   const Error *GetPosCaptureStat( POS_CAPTURE_STAT &stat );
   const Error *GetIndexCapture( int32 &value );
   const Error *GetHomeCapture( int32 &value );
   //@}

   /***************************************************************************/
   /** \name General parameter setup.
   */
   /***************************************************************************/
   //@{
   const Error *GetAmpConfig( AmpConfig &cfg );
   const Error *SetAmpConfig( AmpConfig &cfg );
   const Error *SaveAmpConfig( AmpConfig &cfg );
   const Error *SaveAmpConfig( void );
   const Error *LoadFromFile( const char *name, int &line );

   const Error *GetCanNetworkConfig( CanNetworkConfig &cfg );
   const Error *SetCanNetworkConfig( CanNetworkConfig &cfg );
   const Error *GetAmpInfo( AmpInfo &info );
   const Error *GetMtrInfo( MtrInfo &info );
   const Error *SetMtrInfo( MtrInfo &info );
   const Error *GetRegenConfig( RegenConfig &cfg );
   const Error *SetRegenConfig( RegenConfig &cfg );
   //@}

   /***************************************************************************/
   /** \name Control loop setup
   */
   /***************************************************************************/
   //@{
   const Error *GetPosLoopConfig( PosLoopConfig &cfg );
   const Error *SetPosLoopConfig( PosLoopConfig &cfg );
   const Error *GetVelLoopConfig( VelLoopConfig &cfg );
   const Error *SetVelLoopConfig( VelLoopConfig &cfg );
   const Error *GetCrntLoopConfig( CrntLoopConfig &cfg );
   const Error *SetCrntLoopConfig( CrntLoopConfig &cfg );
   const Error *GetVloopOutputFilter( Filter &f );
   const Error *SetVloopOutputFilter( Filter &f );
   const Error *GetVloopCommandFilter( Filter &f );
   const Error *SetVloopCommandFilter( Filter &f );
   //@}

   /***************************************************************************/
   /** \name Position and velocity windows
   */
   /***************************************************************************/
   //@{
   const Error *SetTrackingWindows( TrackingWindows &cfg );
   const Error *GetTrackingWindows( TrackingWindows &cfg );
   const Error *SetPositionErrorWindow( uunit value );
   const Error *GetPositionErrorWindow( uunit &value );
   const Error *SetPositionWarnWindow( uunit value );
   const Error *GetPositionWarnWindow( uunit &value );
   const Error *SetSettlingWindow( uunit value );
   const Error *GetSettlingWindow( uunit &value );
   const Error *SetSettlingTime( uint16 value );
   const Error *GetSettlingTime( uint16 &value );
   const Error *SetVelocityWarnWindow( uunit value );
   const Error *GetVelocityWarnWindow( uunit &value );
   const Error *SetVelocityWarnTime( uint16 value );
   const Error *GetVelocityWarnTime( uint16 &value );
   const Error *SetSoftLimits( SoftPosLimit &cfg );
   const Error *GetSoftLimits( SoftPosLimit &cfg );
   //@}

   /***************************************************************************/
   /** \name Homing mode.
   */
   /***************************************************************************/
   //@{
   const Error *GoHome( void );
   const Error *GoHome( HomeConfig &cfg );
   const Error *SetHomeConfig( HomeConfig &cfg );
   const Error *GetHomeConfig( HomeConfig &cfg );
   const Error *SetHomeMethod( COPLEY_HOME_METHOD method, uint16 extended=0 );
   const Error *GetHomeMethod( COPLEY_HOME_METHOD &method, uint16 *extended=0 );
   const Error *SetHomeOffset( uunit value  );
   const Error *GetHomeOffset( uunit &value );
   const Error *SetHomeVelFast( uunit value  );
   const Error *GetHomeVelFast( uunit &value );
   const Error *SetHomeVelSlow( uunit value  );
   const Error *GetHomeVelSlow( uunit &value );
   const Error *SetHomeAccel( uunit value  );
   const Error *GetHomeAccel( uunit &value );
   const Error *SetHomeCurrent( int16 value  );
   const Error *GetHomeCurrent( int16 &value  );
   const Error *SetHomeDelay( int16 value  );
   const Error *GetHomeDelay( int16 &value  );
   const Error *GetHomeAdjustment( uunit &value );
   //@}

   /***************************************************************************/
   /** \name Quick stop support
   */
   /***************************************************************************/
   //@{
   const Error *QuickStop( void );
   const Error *HaltMove( void );
   const Error *SetQuickStopDec( uunit value );
   const Error *GetQuickStopDec( uunit &value );
   const Error *SetHaltMode( HALT_MODE mode );
   const Error *GetHaltMode( HALT_MODE &mode );
   const Error *SetQuickStop( QUICK_STOP_MODE mode );
   const Error *GetQuickStop( QUICK_STOP_MODE &mode );
   //@}

   /***************************************************************************/
   /** \name Point to point move support (position profile mode)
   */
   /***************************************************************************/
   //@{
   const Error *SetupMove( ProfileConfigTrap &cfg );
   const Error *SetupMove( ProfileConfigScurve &cfg );
   const Error *SetupMove( ProfileConfigVel &cfg );

   const Error *DoMove( ProfileConfigTrap &cfg, bool relative = false );
   const Error *DoMove( ProfileConfigScurve &cfg, bool relative = false );
   const Error *DoMove( ProfileConfigVel &cfg );
   const Error *DoMove( uunit pos, bool relative = false );
   const Error *StartMove( bool relative = false );

   /// Start an absolute point to point move to the specified position.
   /// This is identical to calling Amp::DoMove( pos )
   /// @param pos The position to move to
   /// @return A pointer to an error object, or NULL on success
   const Error *MoveAbs( uunit pos ){ return DoMove( pos, false ); }

   /// Start a relative point to point move of the specified distance.
   /// This is identical to calling Amp::DoMove( dist, true )
   /// @param dist The distance to move
   /// @return A pointer to an error object, or NULL on success
   const Error *MoveRel( uunit dist ){ return DoMove( dist, true ); }

   const Error *SetProfileConfig( ProfileConfig &cfg );
   const Error *GetProfileConfig( ProfileConfig &cfg );

   const Error *SetProfileType( PROFILE_TYPE type );
   const Error *GetProfileType( PROFILE_TYPE &type );
   const Error *SetTargetPos( uunit value );
   const Error *GetTargetPos( uunit &value );
   const Error *SetProfileVel( uunit value );
   const Error *GetProfileVel( uunit &value );
   const Error *SetProfileAcc( uunit value );
   const Error *GetProfileAcc( uunit &value );
   const Error *SetProfileDec( uunit value );
   const Error *GetProfileDec( uunit &value );
   const Error *SetProfileJerk( uunit value  );
   const Error *GetProfileJerk( uunit &value  );
   //@}

   /***************************************************************************/
   /** \name Profile velocity mode support
   */
   /***************************************************************************/
   //@{
   const Error *SetTargetVel( uunit value );
   const Error *GetTargetVel( uunit &value );
   //@}

   /***************************************************************************/
   /** \name Profile torque mode support
   */
   /***************************************************************************/
   //@{
   const Error *SetTorqueTarget( int16 value );
   const Error *GetTorqueTarget( int16 &value );
   const Error *GetTorqueDemand( int16 &value );
   const Error *GetTorqueActual( int16 &value );
   const Error *SetTorqueSlope( int32 value );
   const Error *GetTorqueSlope( int32 &value );
   const Error *SetTorqueRated( int32 value );
   const Error *GetTorqueRated( int32 &value );
   //@}

   /***************************************************************************/
   /** \name PVT (interpolated position) trajectory support
   */
   /***************************************************************************/
   //@{
   const Error *SendTrajectory( Trajectory &trj, bool start=true );
   const Error *StartPVT( void );
   const Error *GetPvtBuffFree( int16 &n );
   const Error *GetPvtSegID( uint16 &id );
   const Error *GetPvtBuffStat( uint32 &stat );
   const Error *GetPvtSegPos( uunit &pos );
   //@}

   /***************************************************************************/
   /** \name Amplifier event processing
   */
   /***************************************************************************/
   //@{
   const Error *WaitEvent( Event &e, int32 timeout=-1 );
   const Error *WaitEvent( Event &e, int32 timeout, AMP_EVENT &match );
   const Error *WaitMoveDone( int32 timeout=-1 );
   const Error *ClearNodeGuardEvent( void );
   const Error *GetEventMask( AMP_EVENT &e );
   const Error *GetErrorStatus( bool noComm=false );
   //@}

   /***************************************************************************/
   /** \name Unit conversion functions.
     If unit conversions are enabled in CML_Settings.h, then these functions handle
     the details of converting from user position, velocity, acceleration & jerk 
     units to the internal units used by the amplifier.  
     */
   /***************************************************************************/
   //@{
   virtual const Error *SetCountsPerUnit( uunit cts );
   virtual const Error *GetCountsPerUnit( uunit &cts );

   virtual const Error *SetCountsPerUnit( uunit load, uunit mtr );
   virtual const Error *GetCountsPerUnit( uunit &load, uunit &mtr );

   virtual int32 PosUser2Mtr( uunit pos );
   virtual int32 VelUser2Mtr( uunit vel );
   virtual int32 AccUser2Mtr( uunit vel );
   virtual uunit PosMtr2User( int32 pos );
   virtual uunit VelMtr2User( int32 vel );
   virtual uunit AccMtr2User( int32 vel );

   virtual int32 PosUser2Load( uunit pos );
   virtual int32 VelUser2Load( uunit vel );
   virtual int32 AccUser2Load( uunit acc );
   virtual int32 JrkUser2Load( uunit jrk );
   virtual uunit PosLoad2User( int32 pos );
   virtual uunit VelLoad2User( int32 vel );
   virtual uunit AccLoad2User( int32 acc );
   virtual uunit JrkLoad2User( int32 jrk );
   //@}

   /***************************************************************************/
   /** \name Linkage access
     Amplifier object may be attached to a Linkage object.  In this case, multi-
     axis moves may be easily performed through calls to the Linkage object holding
     the amplifiers.
     */
   /***************************************************************************/
   //@{

   /// Return a pointer to the linkage that this amplifier is attached to.
   /// @return The linkage pointer, or NULL if the Amp is not attached to
   ///         any linkage object.
   class Linkage *GetLinkage( void ){ return myLink; }
   //@}

   /***************************************************************************/
   /** \name Non standard modes of operation.
     These functions are used when running in modes other then the standard 
     CANopen position modes.  Note that at present the libraries offer only very
     limited support for these modes.
     */
   /***************************************************************************/
   //@{
   const Error *GetFuncGenConfig( FuncGenConfig &cfg );
   const Error *SetFuncGenConfig( FuncGenConfig &cfg );
   const Error *GetAnalogRefConfig( AnalogRefConfig &cfg );
   const Error *SetAnalogRefConfig( AnalogRefConfig &cfg );
   const Error *GetPwmInConfig( PwmInConfig &cfg );
   const Error *SetPwmInConfig( PwmInConfig &cfg );
   const Error *SetCurrentProgrammed( int16 crnt );
   const Error *GetCurrentProgrammed( int16 &crnt );
   const Error *SetVelocityProgrammed( uunit vel );
   const Error *GetVelocityProgrammed( uunit &vel );
   const Error *SetMicrostepRate( int16 rate );
   const Error *GetMicrostepRate( int16 &rate );
   //@}

   /***************************************************************************/
   /** \name Trace control functions
     The amplifier supports an internal 'tracing' mechanism which allows certain
     internal variables to be sampled at a fixed rate and stored to internal 
     memory.  This mechanism is used by the CME program to implement the oscilloscope
     display of internal amplifier information.  The following methods are available
     to allow this amplifier feature to be used by CML programs.
     */
   /***************************************************************************/
   //@{
   const Error *GetTraceStatus( AMP_TRACE_STATUS &stat, int16 &samp, int16 &sampMax );
   const Error *GetTraceRefPeriod( int32 &per );
   const Error *GetTracePeriod( int16 &per );
   const Error *SetTracePeriod( int16 per );
   const Error *GetTraceTrigger( AMP_TRACE_TRIGGER &type, uint8 &chan, int32 &level, int16 &delay );
   const Error *SetTraceTrigger( AMP_TRACE_TRIGGER type, uint8 chan=0, int32 level=0, int16 delay=0 );
   const Error *GetTraceMaxChannel( uint8 &max );
   const Error *GetTraceChannel( uint8 ndx, AMP_TRACE_VAR &value );
   const Error *SetTraceChannel( uint8 ndx, AMP_TRACE_VAR value );
   const Error *GetTraceData( int32 *data, int32 &max );
   const Error *TraceStart( void );
   const Error *TraceStop( void );
   //@}

protected:

   const Error *GetStatusWord( uint16 &value );
   const Error *SetControlWord( uint16 value );
   const Error *GetControlWord( uint16 &value );

   const Error *ClearEventLatch( EVENT_STATUS stat );

   virtual void HandleStateChange( NodeState from, NodeState to );

   const Error *FormatPvtSeg( int32 pos, int32 vel, uint8 time, uint8 *buff );
   const Error *FormatPtSeg( int32 pos, uint8 time, uint8 *buff );
   const Error *SetPvtInitialPos( int32 pos, bool viaSDO = true );
   const Error *PvtBufferFlush( bool viaSDO = true );
   const Error *PvtBufferPop( uint16 n = 1, bool viaSDO = true );
   const Error *PvtClearErrors( uint8 mask, bool viaSDO = true );

   /// This is an event map that is used to track amplifier 
   /// events and state changes
   EventMap eventMap;

private:
   const Error *SetupSynchPDO( AmpSettings &settings );

   /// Save software version number
   uint16 SwVersionNum; 

   /// Amplifier hardware type
   uint16 hwType;

   /// This event map is used to track the current state of
   /// the general purpose input pins.
   EventMap inputStateMap;

   /// This is a copy of the settings object passed to the 
   /// constructor or Init method.  It's used to re-initialize
   /// the amplifier.
   AmpSettings initialSettings;

   /// A transmit PDO that causes the amplifier to send
   /// it's status word every time it changes.
   TPDO_Status statPdo;

   /// Last value sent down to the control word
   uint16 lastCtrlWord;

   /// Last operation mode sent to node
   AMP_MODE lastMode;

   /// Upper bits (servo or microstep) of the CANopen control
   /// mode to use if not specified.
   AMP_MODE canCtrlMethod;

   /// True if the amplifier is enabled by software.
   bool enabled;

   /// Last home method that was set.
   COPLEY_HOME_METHOD lastHomeMethod;

   /// A transmit PDO used to send PVT buffer count 
   /// information.
   TPDO_PvtBuffStat buffStatPdo;

   /// A receive PDO used to transmit PVT segment 
   /// information to the amplifier
   RPDO_Pvt pvtPdo;

   /// Used to keep track of integrity counter sent with PVT segments
   uint16 pvtSegID;

   /// ID number of PVT segment current in use by the amplifier
   uint16 pvtSegActive;

   // Amplifier's PVT buffer size
   uint16 pvtBuffSize;

   /// Tracks last position value sent in a PVT segment.  This is used
   /// to send relative move segments for large positions.
   int32 pvtLastPos;

   /// If true, I'm pulling trajectory segments out of my cache to handle
   /// a segment sequencing error.  If false, I'm using new segments.
   bool pvtUseCache;

   /// If I'm using cached segments, then this is the ID of the next 
   /// segment I need to pull from the cache.
   uint16 pvtCacheID;

   /// PVT cache object used to keep track of old PVT segments.
   PvtSegCache pvtCache;

   /// Points to the currently running PVT trajectory
   Trajectory *pvtTrj;

#ifdef CML_ENABLE_USER_UNITS
   // Load encoder unit conversion scaling factors
   double u2lPos; ///< Used to convert position from user units to load units
   double u2lVel; ///< Used to convert velocity from user units to load units
   double u2lAcc; ///< Used to convert acceleration from user units to load units
   double u2lJrk; ///< Used to convert jerk from user units to load units
   double l2uPos; ///< Used to convert position from load units to user units
   double l2uVel; ///< Used to convert velocity from load units to user units
   double l2uAcc; ///< Used to convert acceleration from load units to user units
   double l2uJrk; ///< Used to convert jerk from user load to user units

   // Motor encoder unit conversion scaling factors
   double u2mPos; ///< Used to convert position from user units to motor units
   double u2mVel; ///< Used to convert velocity from user units to motor units
   double u2mAcc; ///< Used to convert acceleration from user units to motor units
   double m2uPos; ///< Used to convert position from motor units to user units
   double m2uVel; ///< Used to convert velocity from motor units to user units
   double m2uAcc; ///< Used to convert acceleration from motor units to user units
#endif

   void MoveAborted( void );
   void PvtStatusUpdate( uint32 status );
   const Error *RequestStatUpdt( void );
   friend class TPDO_PvtBuffStat;
   friend class TPDO_Status;

   // The linkage class which owns this Amp uses this pointer
   // to keep track of what linkage the Amp belongs to.
   friend class Linkage;
   void SetLinkage( class Linkage *l ){ myLink = l; }
   class Linkage *myLink;

   /// Private copy constructor (not supported)
   Amp( const Amp& );

   /// Private assignment operator (not supported)
   Amp& operator=( const Amp& );
};

CML_NAMESPACE_END()

#endif


