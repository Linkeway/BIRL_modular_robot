/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
  This file contains the code used by the Amp object
  to stream PVT trajectory profiles over the CANopen 
  network.
  */

#include "CML.h"

CML_NAMESPACE_USE();

/**************************************************
 * PVT buffer errors
 **************************************************/
#define PVTERR_SEQUENCE        0x01
#define PVTERR_OVERFLOW        0x02
#define PVTERR_UNDERFLOW       0x04

/**************************************************
 * PVT buffer status bit mapping
 **************************************************/
#define PVTSTAT_NEXTID         0x0000FFFF
#define PVTSTAT_FREECT         0x00FF0000
#define PVTSTAT_ERROR          0x7F000000
#define PVTSTAT_EMPTY          0x80000000

/***************************************************************************/
/**
  Format a PVT trajectory segment.  The position, velocity and time information
  passed to the function are organized into the proper format and stored in the
  passed buffer.  The buffer is assumed to be at least 8 bytes long.

  @param pos Position at start of segment (encoder counts)
  @param vel Velocity at start of segment (0.1 counts/sec)
  @param time Time till next segment (milliseconds)
  @param buff Points to the buffer where the message will be stored.
  @return An error object
  */
/***************************************************************************/
const Error *Amp::FormatPvtSeg( int32 pos, int32 vel, uint8 time, uint8 *buff )
{
   cml.Debug( "Amp %d PVT Segment: %-10ld %-10ld %3d\n", GetNodeID(), pos, vel, time );

   // Save the lowest 3 bits of the segment ID.  This allows the
   // amplifier to identify missing profile segments.
   buff[0] = 7 & pvtSegID;

   // If position is more then 24 bits then we will have to send a 
   // relative move segment.  I'll just find the difference between 
   // this position and the last one sent.
   if( pos > 0x007FFFFF || -pos > 0x007FFFFF )
   {
      pos -= pvtLastPos;

      // if the result is still more then 24 bits, return an error
      if( pos > 0x007FFFFF || -pos > 0x007FFFFF )
	 return &AmpError::pvtSegPos;

      // mark the segment as relative
      buff[0] |= 0x10;
   }

   // If velocity is more then 24-bits, send it in lower resolution
   if( vel > 0x007FFFFF )
   {
      vel = (vel+50)/100;
      buff[0] |= 0x08;

      if( vel > 0x007FFFFF )
	 return &AmpError::pvtSegVel;
   }
   else if( -vel > 0x007FFFFF )
   {
      vel = (vel-50)/100;
      buff[0] |= 0x08;

      if( -vel > 0x007FFFFF )
	 return &AmpError::pvtSegVel;
   }

   // Format the message and return     
   buff[1] = time;
   buff[2] = pos;
   buff[3] = pos>>8;
   buff[4] = pos>>16;
   buff[5] = vel;
   buff[6] = vel>>8;
   buff[7] = vel>>16;

   return 0;
}

/***************************************************************************/
/**
  Format a PT trajectory segment.  The position and time information
  passed to the function are organized into the proper format and stored in the
  passed buffer.  The buffer is assumed to be at least 8 bytes long.

  @param pos Position at start of segment (encoder counts)
  @param time Time till next segment (milliseconds)
  @param buff Points to the buffer where the message will be stored.
  @return An error object
  */
/***************************************************************************/
const Error *Amp::FormatPtSeg( int32 pos, uint8 time, uint8 *buff )
{
   cml.Debug( "Amp %d PT Segment: %-10ld %3d\n", GetNodeID(), pos, time );

   // Save the lowest 3 bits of the segment ID.  This allows the
   // amplifier to identify missing profile segments.
   buff[0] = 7 & pvtSegID;

   // PT points are always passed using buffer format code 5
   buff[0] |= (5<<3);

   // Format the message and return     
   buff[1] = time;
   buff[2] = ByteCast(pos);
   buff[3] = ByteCast(pos>>8);
   buff[4] = ByteCast(pos>>16);
   buff[5] = ByteCast(pos>>24);

   return 0;
}

/***************************************************************************/
/**
  Set the initial position for a PVT trajectory.  This function sends a full
  32-bit position value which can be used to start a PVT move beyond the 24-bit
  limit of normal segments.  It's normally used at the beginning of a PVT 
  trajectory when the starting position is greater then 24-bits and the commanded
  position at the time of the move's start is not obvious.

  @param pos The 32-bit initial position
  @param viaSDO If true, use a SDO to download the message.  If false, use a PDO.
  default is true.
  @return An error object
  */
/***************************************************************************/
const Error *Amp::SetPvtInitialPos( int32 pos, bool viaSDO )
{
   byte data[8];

   data[0] = ByteCast(7 & pvtSegID);
   data[0] |= 0x20;

   data[1] = ByteCast(pos);
   data[2] = ByteCast(pos>>8);
   data[3] = ByteCast(pos>>16);
   data[4] = ByteCast(pos>>24);

   if( viaSDO )
      return sdo.Download( OBJID_PVT_DATA, 0, 8, data );
   else
      return pvtPdo.Transmit( data );
}

/***************************************************************************/
/**
  Flush the amplifier's PVT trajectory buffer.  Flushing the buffer in this 
  way will cause any running profile to be aborted.
  @param viaSDO If true, use a SDO to download the message.  If false, use a PDO.
  default is true.
  @return An error object.
  */
/***************************************************************************/
const Error *Amp::PvtBufferFlush( bool viaSDO )
{
   byte data[8];

   data[0] = 0x80;
   if( viaSDO ) 
      return sdo.Download( OBJID_PVT_DATA, 0, 8, data );
   else
      return pvtPdo.Transmit( data );
}

/***************************************************************************/
/**
  Clear the specified PVT buffer errors.  
  @param mask A bit mask representing which PVT buffer errors to clear.
  @param viaSDO If true, use a SDO to download the message.  If false, use a PDO.
  default is true.
  @return An error object.
  */
/***************************************************************************/
const Error *Amp::PvtClearErrors( uint8 mask, bool viaSDO )
{
   byte data[8];

   data[0] = 0x82;
   data[1] = ByteCast(mask);

   if( viaSDO )
      return sdo.Download( OBJID_PVT_DATA, 0, 8, data );
   else
      return pvtPdo.Transmit( data );
}

/***************************************************************************/
/**
  Pop the N most recently sent segments off the amplifier's PVT trajectory buffer.
  If there are less then N segments on the buffer, then the buffer is cleared.
  Any profile running on the amplifier will continue to run (is not aborted)
  unless a buffer underflow occurs.
  @param n The number of segments to pop off the buffer.  Defaults to 1.
  @param viaSDO If true, use a SDO to download the message.  If false, use a PDO.
  default is true.
  @return An error object.
  */
/***************************************************************************/
const Error *Amp::PvtBufferPop( uint16 n, bool viaSDO )
{
   byte data[8];

   data[0] = ByteCast(0x81);
   data[1] = ByteCast(n);
   data[2] = ByteCast(n>>8);

   if( viaSDO )
      return sdo.Download( OBJID_PVT_DATA, 0, 8, data );
   else
      return pvtPdo.Transmit( data );
}

/***************************************************************************/
/**
  Upload a PVT move trajectory to the amplifier and optionally start the move.

  @param trj Reference to the trajectory that will be feed to the amp.  A local
  pointer to this trajectory will be stored if the entire profile will 
  not fit in the amplifiers on-board buffer.  This pointer will be kept
  until the entire profile has been uploaded to the amp.  It is therefore 
  important to ensure that the trajectory object will remain valid (i.e. not be
  deallocated) until the amplifier object has called the Trajectory.Finish() 
  method on it.

  @param start If true (the default), the profile will be started by this call.
  If false, the profile will be uploaded, but not started.  Use true if
  this is a single axis move, false if this is part of a multi-axis move
  which needs to be synchronized.

  @return An error object.
  */
/***************************************************************************/
const Error *Amp::SendTrajectory( Trajectory &trj, bool start )
{
   const Error *err;

   // Make sure we are in interpolated position mode
   err = SetAmpMode( AMPMODE_CAN_PVT );
   if( err ) return err;

   // Get the trajectory buffer status value.
   uint32 stat;
   err = GetPvtBuffStat( stat );
   if( err ) return err;

   // Clear any buffer errors that are outstanding.
   if( stat & PVTSTAT_ERROR )
   {
      err = PvtClearErrors( (uint8)( (stat & PVTSTAT_ERROR) >> 24 ) );
      if( !err ) err = GetPvtBuffStat( stat );
      if( err ) return err;
   }

   // We expect the buffer to be empty at the start of the move.
   // If it isn't we'll clear it.
   if( !(stat & PVTSTAT_EMPTY) )
   {
      err = PvtBufferFlush();
      if( !err ) err = GetPvtBuffStat( stat );
      if( err ) return err;
   }

   // See how many segments will fit in the buffer (which should be
   // empty at this point).  I need at least 2 and would normally
   // expect many more.  If for some reason it's less then 2, then
   // I'll just return an error.  This really should never happen.
   uint8 n = (uint8)((stat & PVTSTAT_FREECT) >> 16);
   if( n < 2 )
      return &AmpError::pvtBufferFull;

   pvtBuffSize = n;

   /// Clear the PVT segment cache
   pvtUseCache = false;
   pvtCache.Clear();

   /// Make sure the trajectory object is ready to go
   err = trj.StartNew();
   if( err ) return err;

   // Find the maximum number of buffer points that the trajectory
   // want's us to use.  I'll limit the number of points I download
   // to this value.
   int max = trj.MaximumBufferPointsToUse();
   if( n > max ) n = max;

   // Keep track of the first segment being sent
   pvtSegActive = pvtSegID = (stat & PVTSTAT_NEXTID);

   /**************************************************
    * Upload as many segments as possible.  I use an
    * SDO to upload the segments here, but will use a 
    * PDO to upload the rest of the profile once we
    * start moving.  This simplifies my error handling
    * here when I can afford the overhead of the SDO.
    **************************************************/
   uint8 segBuff[8];
   uint8 time;
   for( uint8 i=0; i<n; i++ )
   {
      uunit p,v;

      // See if this is a PVT or PT segment
      bool useVel = trj.UseVelocityInfo();

      err = trj.NextSegment( p, v, time);
      if( err ) break; 

      int32 pos = PosUser2Load(p);

      int32 vel = 0;
      if( useVel ) vel = VelUser2Load(v);

      // If this is the first segment of the move, then 
      // make sure my initial position is known.  I really
      // only need to do this if the position value is
      // larger then 24 bits, otherwise I'll be sending it
      // as an absolute value and won't need the previous
      // position anyway.
      if( useVel && !i && (pos > 0x007FFFFF || -pos > 0x007FFFFF) )
      {
	 err = SetPvtInitialPos( pos );
	 if( err ) break;
	 pvtSegID++;
	 n--;
      }

      // Format the segment
      if( useVel )
	 err = FormatPvtSeg( pos, vel, time, segBuff );
      else
	 err = FormatPtSeg( pos, time, segBuff );
      if( err ) break;

      // Send it using an SDO
      //		err = sdo.Download( OBJID_PVT_DATA, 0, 8, segBuff );
      //		if( err ) break;

      // Send it using a PDO
      pvtPdo.Transmit( segBuff );
      pvtCache.AddSegment( segBuff, pvtSegID, p );

      // Update the segment ID counter and the last segment
      // position info.
      pvtSegID++;
      pvtLastPos = pos;

      // If time is zero, this is the last segment in the move.
      if( !time ) break;
   }

   // If the trajectory object indicated that there were no segments available, then
   // simply ignore this error.  This error code is used to limit the amount of data
   // sent to the amplifier.
   if( err == &TrjError::NoneAvailable )
      err = 0;

   // Get the buffer status.  We'll process this later
   if( !err ) err = GetPvtBuffStat( stat );

   // If an error occurred during the download, flush 
   // the buffer and return the error code.
   if( err )
   {
      trj.Finish();
      PvtBufferFlush();
      return err;
   }

   // If the whole profile hasn't been sent, keep a pointer
   // to the trajectory object so I can spool it up to the 
   // amp as buffer space becomes available.
   if( time )
      pvtTrj = &trj;
   else
   {
      trj.Finish();
      pvtTrj = 0;
   }

   // Process the current buffer status to handle any lost messages.
   PvtStatusUpdate( stat );

   // Start the move if so requested
   if( start )
      err = StartPVT();

   return err;
}

/***************************************************************************/
/**
  Start a PVT move that has already been uploaded.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::StartPVT( void )
{
   // Start the move
   const Error *err = StartMove();

   if( err && pvtTrj )
   {
      pvtTrj->Finish();
      pvtTrj = 0;
   }

   return err;
}

/***************************************************************************/
/**
  A new abort event was just received.  This function is called from the 
  status PDO when the 'move aborted' status is set.  It's used to clean up
  the PVT move in progress.
  */
/***************************************************************************/
void Amp::MoveAborted( void )
{
   if( pvtTrj )
   {
      pvtTrj->Finish();
      pvtTrj = 0;
   }
}

/***************************************************************************/
/**
  Get the starting position of the PVT segment currently active in the 
  amplifier.  When running in PVT mode, this allows an approximation of the
  amplifier position to be retrieved without adding any additional overhead
  to the CANopen network.

  The position returned by this function is only valid when running in PVT mode.

  @param pos The position is returned here.
  @return A pointer to an error object, or NULL on success.
  */
/***************************************************************************/
const Error *Amp::GetPvtSegPos( uunit &pos )
{
   if( pvtCache.GetPosition( &pos, pvtSegActive ) )
      return 0;

   return &AmpError::pvtPosUnavail;
}

/***************************************************************************/
/**
  This function is called by the PVT status PDO receiver function.  It takes
  care of the gory details of streaming out a trajectory that's too big to 
  fit in the amplifier's PVT buffer all at once.

  Note that this function will be called by the receive task and therefore 
  must be as quick as possible.  Also, SDO access is not available from within
  the receive task.

  @param status The amplifier status word passed in the PDO
  */
/***************************************************************************/
void Amp::PvtStatusUpdate( uint32 status )
{
   // Break the status into it's main components.  These are:
   //
   // - The segment ID of the next segment expected by the amp
   // - The number of free positions in the buffer
   // - Any error codes
   uint16 ampNextID = (uint16)(status & PVTSTAT_NEXTID);
   int    freeCt    = (uint8)((status & PVTSTAT_FREECT)>>16);
   uint8  errors    = (uint8)((status & PVTSTAT_ERROR )>>24);

   cml.Debug( "Amp %d PVT Stat: %5u %3u 0x%02x\n", GetNodeID(), ampNextID, freeCt, errors );

   // Set an amplifier status bit if the PVT buffer is empty
   if( status & 0x80000000 )
      eventMap.setBits( AMPEVENT_PVT_EMPTY );
   else
      eventMap.clrBits( AMPEVENT_PVT_EMPTY );

   // Keep track of the active segment ID
   pvtSegActive = ampNextID - pvtBuffSize + freeCt - 1;

   // If the amplifier has experienced an underflow error, then
   // it has already aborted the trajectory.  At this point there
   // is no reason to send more segments, so I'll just return.
   if( errors & PVTERR_UNDERFLOW )
   {
      if( pvtTrj )
      {
	 pvtTrj->Finish();
	 pvtTrj = 0;
      }
      return;
   }

   // If a segment sequencing error has occurred, then I'll try
   // to resend data starting at the segment the amp desires.
   // First, I'll clear the error and update my state to
   // indicate that segments need to be resent.
   // That's all I'll do until I get a status indicating that 
   // the error was cleared since there could be other pending
   // status messages coming my way with the old error flagged, and
   // I don't want to get confused about where the error occurred.
   if( errors & PVTERR_SEQUENCE )
   {
      PvtClearErrors( PVTERR_SEQUENCE, false );
      pvtUseCache = true;
      pvtCacheID = ampNextID;
      return;
   }

   // OK, no errors that we care about.  Reduce the free count value
   // passed in the status to take any recently sent segments into
   // account.  The result is the number of new segments we can
   // send safely.
   if( pvtUseCache )
      freeCt -= (pvtCacheID - ampNextID);
   else
      freeCt -= (pvtSegID - ampNextID);

   // Reduce the number of messages to send based on the trajectories
   // desired buffer usage.
   if( pvtTrj )
   {
      int max = pvtTrj->MaximumBufferPointsToUse();
      if( max < pvtBuffSize )
	 freeCt -= (pvtBuffSize-max);
   }

   // Limit the number of new segments I'll send in response
   // to a single status message.  
   if( freeCt > initialSettings.maxPvtSendCt )
      freeCt = initialSettings.maxPvtSendCt;

   // Now, send as many segments as possible.
   for( uint8 i=0; i<freeCt; i++ )
   {
      uint8 buff[8];

      // See if we need to resend an old segment.  
      // If so, I'll pull it out of the cache.
      if( pvtUseCache )
      {
	 // If the necessary segment isn't available, then 
	 // I have no choice but to flush the cache and 
	 // abort the profile.
	 if( !pvtCache.GetSegment( buff, pvtCacheID ) )
	 {
	    PvtBufferFlush( false );
	    if( pvtTrj )
	    {
	       pvtTrj->Finish();
	       pvtTrj = 0;
	    }
	    return;
	 }

	 if( ++pvtCacheID == pvtSegID )
	    pvtUseCache = false;
      }

      // We need to get a new segment from the trajectory generator.
      // If there isn't one then it means that we're done sending
      // segments out, so we can just return.
      else if( !pvtTrj ) 
	 return;

      else
      {
	 uint8 time;
	 uunit p, v;

	 // See if this is a PVT or PT segment
	 bool useVel = pvtTrj->UseVelocityInfo();

	 // If the trajectory generator returns an error then I'll 
	 // just return and try again next time.
	 const Error *err = pvtTrj->NextSegment( p, v, time );
	 if( err )
	 {
	    if( err != &TrjError::NoneAvailable )
	       cml.Warn( "Amp %d PVT NextSegment returned %s\n", err->toString() );
	    return;
	 }

	 int32 pos = PosUser2Load(p);
	 int32 vel = 0;
	 if( useVel ) vel = VelUser2Load(v);

	 // Format the PVT segment
	 if( useVel )
	    err = FormatPvtSeg( pos, vel, time, buff );
	 else
	    err = FormatPtSeg( pos, time, buff );

	 // If the format failed, this is a critical error.
	 // I'll flush the trajectory buffer which causes the
	 // trajectory to be aborted.
	 if( err )
	 {
	    PvtBufferFlush( false );
	    pvtTrj->Finish();
	    pvtTrj = 0;
	    return;
	 }

	 // Add this segment to my cache for later error handling
	 pvtCache.AddSegment( buff, pvtSegID, p );

	 pvtSegID++;
	 pvtLastPos = pos;

	 // If this was the last segment in the trajectory (i.e. the
	 // time was zero), then discard the trajectory object.
	 if( !time )
	 {
	    pvtTrj->Finish();
	    pvtTrj = 0;
	 }
      }

      // Send the segment using a PDO
      pvtPdo.Transmit( buff );
   }

   return;
}

/***************************************************************************/
/**
  Add the passed segment to the cache.  Segments must be passed in order and 
  with no gaps between ID numbers.  If this segment doesn't follow those rules
  then the cache will be cleared before the segment is added.
  @param seg Points to an array of 8 bytes which make up the segment to be added.
  The segment data is copied into the cache.  No copy of the pointer is 
  kept locally.
  @param id The ID number of the passed segment.
  @param p The position corresponding to this segment.
  */
/***************************************************************************/
void PvtSegCache::AddSegment( uint8 *seg, uint16 id, uunit p )
{
   // If the ID is not one greater then the last ID
   // added to the cache, then clear it.  
   // Segments must be added to the cache in order 
   // with no gaps.

   if( id != oldest+ct )
   {
      Clear();
      oldest = id;
   }

   // Figure out where to put this one
   int index;

   if( ct < PVTCACHESIZE )
      index = ct++;
   else
   {
      oldest++;
      index = top++;
      if( top == PVTCACHESIZE )
	 top = 0;
   }

   // Add the new segment
   for( int i=0; i<8; i++ )
      cache[index][i] = seg[i];
   pos[index] = p;
}

/***************************************************************************/
/**
  Get the specified segment from the cache.  If the requested segment is 
  available, it's contents will be copied to the passed pointer.
  @param seg A pointer to an array of 8 bytes where the segment data 
  will be copied on success.
  @param id The ID number of the segment being requested
  @return true on success, false if the requested segment isn't available.
  */
/***************************************************************************/
bool PvtSegCache::GetSegment( uint8 *seg, uint16 id )
{
   int16 index = id - oldest;

   // Fail if the index is outside my range
   if( index < 0 || index >= ct )
      return false;

   index += top;
   if( index >= PVTCACHESIZE )
      index -= PVTCACHESIZE;

   for( int i=0; i<8; i++ )
      seg[i] = cache[index][i];

   return true;
}

/***************************************************************************/
/**
  Get the position corresponding to the specified segment from the cache.
  If the requested position is available, it will be copied to the passed pointer.
  @param p A pointer to where the position information will be copied.
  @param id The ID number of the segment being requested
  @return true on success, false if the requested segment isn't available.
  */
/***************************************************************************/
bool PvtSegCache::GetPosition( uunit *p, uint16 id )
{
   int16 index = id - oldest;

   // Fail if the index is outside my range
   if( index < 0 || index >= ct )
      return false;

   index += top;
   if( index >= PVTCACHESIZE )
      index -= PVTCACHESIZE;

   *p = pos[index];
   return true;
}

/***************************************************************************/
/**
  Initialize a transmit PDO used to send PVT buffer status updates
  The PDO is initialized to transmit a PDO on PVT buffer status events.
  The events occur every time a segment is read out of the buffer
  or an error bit is set.

  @param amp Reference to the amplifier object 
  @param slot TPDO slot number for this PDO
  @return An error object
  */
/***************************************************************************/
const Error *TPDO_PvtBuffStat::Init( Amp &amp, uint16 slot )
{
   ampPtr = &amp;

   // Initialize the transmit PDO
   const Error *err = TPDO::Init( amp.GetCanOpen(), amp.GetTpdoCobID( slot ) );

   // Set transmit type to transmit on events
   if( !err ) err = SetType( 255 );

   // Init the stat mapped object
   if( !err ) err = stat.Init( OBJID_PVT_BUFF_STAT, 0 );

   // Add the mapped variable
   if( !err ) err = AddVar( stat );

   // Enable reception of this PDO's messages
   if( !err ) err = EnableReceiver();

   // Program this PDO in the amp, and enable it
   if( !err ) err = amp.PdoSet( slot, *this );

   return err;
}

/***************************************************************************/
/**
  Handle the reception of status information.  This PDO waits for one of the
  required bits in the status word to be set, and posts the amplifiers 
  semaphore when it is.
  */
/***************************************************************************/
void TPDO_PvtBuffStat::Received()
{
   ampPtr->PvtStatusUpdate( stat.Read() );
}

/***************************************************************************/
/**
  Initialize the receive PDO used to pass PVT segments to an amplifier.
  @param amp The amplifier that the PDO is associated with
  @param slot The RPDO slot used by this PDO
  @return An error object
  */
/***************************************************************************/
const Error *RPDO_Pvt::Init( Amp &amp, uint16 slot )
{
   ampPtr = &amp;

   const Error *err = RPDO::Init( amp.GetRpdoCobID(slot) );

   if( !err ) err = dat.Init( OBJID_PVT_DATA, 0, 64 );
   if( !err ) err = AddVar( dat );
   if( !err ) err = SetType( 255 );

   if( !err ) err = amp.PdoSet( slot, *this );

   return err;
}

/***************************************************************************/
/**
  Transmit the passed data over the PDO
  @param data Pointer to the data to send
  @return An error object
  */
/***************************************************************************/
const Error *RPDO_Pvt::Transmit( byte *data )
{
   dat.Set(data);

   return RPDO::Transmit( ampPtr->GetCanOpen() );
}


