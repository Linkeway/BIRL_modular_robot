/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file contains code that implements PDO objects used by the
Copley Controls amplifier object.
*/

#include "CML.h"

CML_NAMESPACE_USE();

/****************************************************************
* CANopen drive status bits (CANopen 402 Object 0x6041)
*****************************************************************/
#define DRIVESTAT_RTSO                      0x0001   // Ready to switch on
#define DRIVESTAT_ON                        0x0002   // Switched on
#define DRIVESTAT_ENA                       0x0004   // Operation enabled
#define DRIVESTAT_FAULT                     0x0008   // A fault has occurred on the drive
#define DRIVESTAT_VENA                      0x0010   // Voltage enabled
#define DRIVESTAT_QSTOP                     0x0020   // Doing quick stop when CLEAR
#define DRIVESTAT_SOD                       0x0040   // Switch-on disabled
#define DRIVESTAT_WARN                      0x0080   // A warning condition is in effect
#define DRIVESTAT_ABORT                     0x0100   // Trajectory aborted
#define DRIVESTAT_REMOTE                    0x0200   // Controlled by CANopen network if set
#define DRIVESTAT_MOVEDONE                  0x0400   // Target reached
#define DRIVESTAT_LIMIT                     0x0800   // Internal limit active
#define DRIVESTAT_SPACK                     0x1000   // pp mode - setpoint acknowledge
#define DRIVESTAT_TRACKERR                  0x2000   // pp mode - following error
#define DRIVESTAT_REFERENCED                0x1000   // home mode - drive has been referenced
#define DRIVESTAT_HOMEERR                   0x2000   // home mode - homing error
#define DRIVESTAT_TRJ                       0x4000   // Trajectory active when set
#define DRIVESTAT_HOMECAP                   0x8000   // Home position captured (requires 4.77+ firmware)

// Event status bits that are considered amplifier errors
#define ERROR_EVENTS (ESTAT_SHORT_CRCT | ESTAT_AMP_TEMP | ESTAT_MTR_TEMP | ESTAT_ENCODER_PWR | \
                      ESTAT_PHASE_ERR | ESTAT_TRK_ERR | ESTAT_UNDER_VOLT | ESTAT_OVER_VOLT)

/***************************************************************************/
/**
Initialize a transmit PDO used to send status updates.
The PDO is initialized to transmit a PDO on status events
which occur when the amp status changes.

@param amp Reference to the amplifier object 
@param slot TPDO slot number for this PDO
@return An error object
*/
/***************************************************************************/
const Error *TPDO_Status::Init( Amp &amp, uint16 slot )
{
   cml.Debug( "Amp %d Status PDO Init\n", amp.GetNodeID() );
   ampPtr = &amp;

   // Initialize the transmit PDO
   const Error *err = TPDO::Init( amp.GetCanOpen(), amp.GetTpdoCobID( slot ) );

   // Set transmit type to transmit on events
   if( !err ) 
      err = SetType( 255 );

   // Let the various mapped variables know which 
   // objects in the amp's object dictionary they
   // are linked to.
   if( !err ) err = status.Init( OBJID_STATUS, 0 );
   if( !err ) err = estat.Init( OBJID_EVENT_STAT, 0 );
   if( !err ) err = inputs.Init( OBJID_INPUTS, 0 );

   // Add the mapped variables
   if( !err ) err = AddVar( status );
   if( !err ) err = AddVar( estat );
   if( !err ) err = AddVar( inputs );

   // Enable reception of this PDOs messages
   if( !err ) err = EnableReceiver();

   // Set the amplifier's event map to the uninitialized state
   ampPtr->eventMap.setMask( AMPEVENT_NOT_INIT );

   // Program this PDO in the amp, and enable it
   if( !err ) err = amp.PdoSet( slot, *this );

   return err;
}

/***************************************************************************/
/**
Handle the reception of status information.  This PDO waits for one of the
required bits in the status word to be set, and posts new events to the 
amplifier as they occur.
*/
/***************************************************************************/
void TPDO_Status::Received()
{
   uint16 s = status.Read();
   uint32 e = estat.Read();
   uint16 i = inputs.Read();
   cml.Debug( "Amp %d status 0x%08x 0x%04x 0x%04x\n", ampPtr->GetNodeID(), e, s, i );
   UpdateEvents( s, e, i );
}

/***************************************************************************/
/**
Update the amplifier's event map based on the status information received
by this PDO.

@param stat The device status word
*/
/***************************************************************************/
void TPDO_Status::UpdateEvents( uint16 stat, uint32 events, uint16 inputs )
{
   uint32 mask = 0;

   if(  stat & DRIVESTAT_SPACK    ) mask |= AMPEVENT_SPACK;
   if(  stat & DRIVESTAT_MOVEDONE ) mask |= AMPEVENT_MOVEDONE;
   if( ~stat & DRIVESTAT_TRJ      ) mask |= AMPEVENT_TRJDONE;
   if( ~stat & DRIVESTAT_QSTOP    ) mask |= AMPEVENT_QUICKSTOP;
   if(  stat & DRIVESTAT_ABORT    ) mask |= AMPEVENT_ABORT;
   if(  stat & DRIVESTAT_HOMECAP  ) mask |= AMPEVENT_HOME_CAPTURE;

   if( events & ERROR_EVENTS      ) mask |= AMPEVENT_ERROR;
   if( events & ESTAT_FAULT       ) mask |= AMPEVENT_FAULT;
   if( events & ESTAT_TRK_WARN    ) mask |= AMPEVENT_POSWARN;
   if( events & ESTAT_TRK_WIN     ) mask |= AMPEVENT_POSWIN;
   if( events & ESTAT_VEL_WIN     ) mask |= AMPEVENT_VELWIN;
   if( events & ESTAT_PWM_DISABLE ) mask |= AMPEVENT_DISABLED;
   if( events & ESTAT_POSLIM      ) mask |= AMPEVENT_POSLIM;    
   if( events & ESTAT_NEGLIM      ) mask |= AMPEVENT_NEGLIM;
   if( events & ESTAT_SOFTLIM_POS ) mask |= AMPEVENT_SOFTLIM_POS;
   if( events & ESTAT_SOFTLIM_NEG ) mask |= AMPEVENT_SOFTLIM_NEG;
   if( events & ESTAT_SOFT_DISABLE) mask |= AMPEVENT_SOFTDISABLE;

   // On new move aborts, do some clean up.
   if( mask & AMPEVENT_ABORT )
   {
      uint32 old = ampPtr->eventMap.getMask();

      if( !(old & AMPEVENT_ABORT) )
	 ampPtr->MoveAborted();
   }

   // Do the same thing if the amplifier is disabled while 
   // a trajectory was in progress
   if( mask & AMPEVENT_DISABLED )
   {
      uint32 old = ampPtr->eventMap.getMask();

      if( !(old & AMPEVENT_TRJDONE) )
	 ampPtr->MoveAborted();
   }

   // Change the bits that this function is responsible for.
   // for now, that's everything but the node guarding 
   // and PVT buffer empty bits.
   ampPtr->eventMap.changeBits( ~(AMPEVENT_PVT_EMPTY|AMPEVENT_NODEGUARD), mask );

   // Update the input pins state
   ampPtr->inputStateMap.setMask( (uint32)inputs );
}

/***************************************************************************/
/**
Transmit PDO used to send out high resolution time stamp messages.
By default, the synch producer is configured to generate this PDO
every 100 ms.
*/
/***************************************************************************/
class TPDO_HighResTime: public TPDO
{
   Pmap32 stamp;                      ///< Used to map the high resolution time stamp information
public:
   /// Default constructor for a high res timestamp transmit PDO
   TPDO_HighResTime(){}
   const Error *Init( Amp &amp, uint16 slot, uint32 id );
};

/***************************************************************************/
/**
Receive PDO used to receive high resolution time stamp messages.
By default, all synch consumers are configured to receive this PDO.
*/
/***************************************************************************/
class RPDO_HighResTime: public RPDO
{
   Pmap32 stamp;                      ///< Used to map the high resolution time stamp information
public:
   RPDO_HighResTime(){}
   const Error *Init( Amp &amp, uint16 slot, uint32 id );
};

/***************************************************************************/
/**
Initialize a transmit PDO used to send high resolution time information.
The PDO is initialized to transmit a PDO every 10 SYNCH messages.
@param amp Reference to the amplifier object 
@param slot TPDO slot number for this PDO
@param id The CAN message ID to be used by this PDO.
@return An error object
*/
/***************************************************************************/
const Error *TPDO_HighResTime::Init( Amp &amp, uint16 slot, uint32 id )
{
   // Initialize the transmit PDO
   const Error *err = TPDO::Init( amp.GetCanOpen(), id );

   // Set transmit type to transmit every 10 synch signals
   if( !err ) err = SetType( 10 );

   // Initialize my time stamp variable to identify the
   // high res time stamp object.
   if( !err ) err = stamp.Init( 0x1013, 0 );

   // Add the mapped variable
   if( !err ) err = AddVar( stamp );

   // Program this PDO in the amp, and enable it
   if( !err ) err = amp.PdoSet( slot, *this );

   return err;
}

/***************************************************************************/
/**
Initialize a receive PDO used to receive high resolution time information.
@param amp The amplifier that the PDO is associated with
@param slot The RPDO slot used by this PDO
@param id The CAN ID value to be used by the PDO
@return An error object
*/
/***************************************************************************/
const Error *RPDO_HighResTime::Init( Amp &amp, uint16 slot, uint32 id )
{
   const Error *err = RPDO::Init( id );

   if( !err ) err = stamp.Init( 0x1013, 0 );
   if( !err ) err = AddVar( stamp );
   if( !err ) err = SetType( 255 );
   if( !err ) err = amp.PdoSet( slot, *this );

   return err;
}


/***************************************************************************/
/**
Setup the amplifier's high resolution time stamp PDO.  This is used to 
synchronize multiple amplifiers over the CANopen network.

To use this PDO, one amplifier (normally the SYNC producer) is configured 
to transmit the PDO roughly every 100ms.  The other amplifiers are configured
to receive the PDO and adjust their internal clocks based on the time stamp
information is contains.
*/
/***************************************************************************/
const Error *Amp::SetupSynchPDO( AmpSettings &settings )
{
   // This feature can be disabled by setting the ID to zero.
   // It's not clear why that would be useful however.
   if( !settings.timeStampID )
      return 0;

   // If this is the synch producer, init the 
   // high resolution time stamp TPDO
   if( settings.synchProducer )
   {
      TPDO_HighResTime pdo;
      return pdo.Init( *this, 4, settings.timeStampID );
   }

   // Otherwise, init an RPDO for this purpose
   else
   {
      RPDO_HighResTime pdo;
      return pdo.Init( *this, 4, settings.timeStampID );
   }
}

