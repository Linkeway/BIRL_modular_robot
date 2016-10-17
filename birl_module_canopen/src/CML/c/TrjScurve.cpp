/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML_Settings.h"
#ifdef CML_ALLOW_FLOATING_POINT

#include <math.h>
#include "CML.h"

CML_NAMESPACE_USE();

CML_NEW_ERROR( ScurveError, BadParam,  "An illegal input parameter was passed" );
CML_NEW_ERROR( ScurveError, NoCalc,    "Trjaectory has not been calculated" );
CML_NEW_ERROR( ScurveError, InUse,     "Trajectory is currently in use" );
CML_NEW_ERROR( ScurveError, NotInUse,  "Trajectory has not been started" );

/***************************************************************************/
/**
S-curve trajectory default constructor.  This simply sets the profile to 
zero length with a starting position of zero.
*/
/***************************************************************************/
TrjScurve::TrjScurve( void )
{
   start = 0;
   inUse = false;
   init = false;
}

/***************************************************************************/
/**
Set the trajectory starting position.  S-curve profiles are internally stored
as absolute moves of some length.  This allows them to be used multiple times 
with different starting positions.  

This function may be used to update the starting position of the trajectory.

@param s The new starting position
*/
/***************************************************************************/
void TrjScurve::SetStartPos( uunit s )
{
   start = s;
   return;
}

/***************************************************************************/
/**
Return the current starting position of the trajectory.  The starting position
will either be the value set using TrjScurve::SetStartPos, or the value set 
using TrjScurve::Calculate.  If neither has been called since construction, 
then the starting position will be zero.

@return The trajectory starting position.
*/
/***************************************************************************/
uunit TrjScurve::GetStartPos( void )
{
   return start;
}

/***************************************************************************/
/**
Calculate a new S-curve profile and also set it's starting position.

@param start The profile starting position.
@param end The profile ending position.
@param vel The maximum allowable velocity for the move.
@param acc The maximum allowable acceleration for the move.
@param dec The maximum allowable deceleration for the move.
@param jrk The maximum jerk (rate of change of acceleration) for the move.

@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *TrjScurve::Calculate( uunit start, uunit end, uunit vel, uunit acc, uunit dec, uunit jrk )
{
   uunit dist = end-start;
   const Error *err = Calculate( dist, vel, acc, dec, jrk );
   if( err ) return err;
   SetStartPos( start );
   return 0;
}

/***************************************************************************/
/**
Calculate a new S-curve profile.  The resulting profile may then be sent 
to an Amp object using the Amp::SendTrajectory method.

Note, all profile parameters are passed in 'user units'.  See the 
documentation for Amp::SetCountsPerUnit for details.

Note also that this function calculate the profile as an absolte move from
the starting position that is set using TrjScurve::SetStartPos.  The same
profile may be used multiple times with different starting positions without
calling the TrjScurve::Calculate function again.

@param dist The distance to move.
@param maxVel The maximum allowable velocity for the move.
@param maxAcc The maximum allowable acceleration for the move.
@param maxDec The maximum allowable deceleration for the move.
@param maxJrk The maximum jerk (rate of change of acceleration) for the move.

@return A pointer to an error object, or NULL on success.

The distance will always be met exactly.  This may be either positive or
negative.
  
The velocity, acceleration, deceleration and jerk values are constraints 
and won't be exceeded.  These must all be positive numbers greater then zero.
*/
/***************************************************************************/
const Error *TrjScurve::Calculate( uunit dist, uunit maxVel, uunit maxAcc, uunit maxDec, uunit maxJrk )
{
   /**************************************************
    * Do some sanity checks on the input limits.
    **************************************************/
   if( maxVel<=0 || maxAcc<=0 || maxDec<=0 || maxJrk<=0 )
      return &ScurveError::BadParam;

   if( inUse ) return &ScurveError::InUse;

   /**************************************************
    * Convert to floating point units if necessary
    **************************************************/
#ifdef CML_ENABLE_USER_UNITS
   P = dist;
   V = maxVel;
   A = maxAcc;
   D = maxDec;
   J = maxJrk;
#else
   P = dist;
   V = maxVel * 0.1;
   A = maxAcc * 10.0;
   D = maxDec * 10.0;
   J = maxJrk * 100.0;
#endif

   /**************************************************
    * Assume positive moves for simplicity.  We'll fix
    * this in the end if necessary.
    **************************************************/
   bool negMove = (P<0);
   if( negMove )
      P *= -1.0;

   if( P==0 )
   {
      tj = tk = ta = td = tv = 0;
      init = true;
      return 0;
   }

   /**************************************************
    * Make sure A <= D.  This reduces the number of 
    * tests I need to do later.  I'll fix this at the
    * end of the calculation.
    **************************************************/
   bool swapAD = (A > D);

   if( swapAD )
   {
      double tmp;
      tmp = A; A = D; D = tmp;
   }

   /**************************************************
    * I'll lower jerk to ensure that my jerk segments
    * are at least 1 millisecond long.
    **************************************************/
   if( J > A*1e3 ) J = A * 1e3;
   if( J > V*1e6 ) J = V * 1e6;
   if( J > P*5e8 ) J = P * 5e8;

   /**************************************************
    * These are the key variables I'll need to find.
    *   tj = time to increase/decrease acceleration
    *   ta = time to run at constant accel
    *   tv = time to run at constant velocity
    *   td = time to run at constant decel
    *   tk = time to increase/decrease deceleration
    **************************************************/

   /**************************************************
    * See if a simple jerk limited move will handle 
    * this.  In this case, the move will be 4 segments
    * each of the same time.
    **************************************************/
   tj = pow( P/(2*J), 1.0/3.0 );
   if( (J*tj < A) && (J*tj*tj < V) )
   {
      ta = td = tv = 0;

      // Adjust time & Jerk to the next higher millisecond
      tk = tj = 0.001 * ceil( tj*1000 );

      J  = P / (2 * tj*tj*tj);
   }

   /**************************************************
    * We know we'll hit either the accel or velocity 
    * limit.  See if the accel limit is too high.
    * If so, this must be a Jerk & Velocity move.
    **************************************************/
   else if( J*V < A*A )
   {
      ta = td = 0;
      tj = sqrt( V/J );

      // Adjust the times so they are integer multiples
      // of milliseconds.  I'll adjust J & V to compensate
      tk = tj = 0.001 * ceil( tj*1000 );
      tv = P/V - 2*tj;

      tv = 0.001 * ceil( tv*1000 );

      V = P / (tv + 2*tj);
      J = V / (tj*tj);
   }

   else 
   {
      /**************************************************
       * At this point we know we will hit the accel limit.
       * We may or may not hit the velocity & decel limits.
       * I'll start by assuming that I'll hit the velocity
       * limit.
       **************************************************/
      double vj, vk;

      tj = A/J;
      vj = A*tj / 2.0;

      ta = (V - 2*vj) / A;

      if( J*V < D*D )
      {
	 td = 0.0;
	 tk = sqrt(V/J);
	 vk = V/2;
      }
      else
      {
	 tk = D/J;
	 td = (V-J*tk*tk) / D;
	 vk = D*tk / 2.0;
      }

      // Find the distance moved getting up to 
      // and down from V
      double pa = tj*vj*2 + ta*vj + A*ta*tj + ta*ta*A/2.0;
      double pd = tk*vk*2 + td*vk + D*td*tk + td*td*D/2.0;

      // If this distance is less then the total move,
      // then I've found my solution.  Otherwise, the
      // velocity limit isn't reached.
      if( pa+pd <= P )
	 tv = (P-pa-pd) / V;

      else
      {
	 /**************************************************
	  * At this point, we know we will hit the accel 
	  * limit, but not the velocity limit.  The only 
	  * question now is whether the decel limit will 
	  * be reached.
	  *
	  * I'll try no decel limit first.
	  **************************************************/
	 tv = 0.0;
	 tk = (sqrt( sqrt(2*P*A)*4*J +A*A ) - A) / (2*J);

	 if( J*tk <= D )
	 {
	    ta = (J*tk*tk - J*tj*tj) / A;
	    td = 0.0;
	 }

	 else
	 {
	    tk = D/J;

	    double a = J*A*(D+A);
	    double b = 3*A*A*D -2*A*D*D +2*A*A*A +3*A*D*D;
	    double c = (A*A + D*D + 2*A*D) * A*A/J - 2*P*J*D;

	    ta = (-b + sqrt(b*b -4*a*c)) / (2*a);
	    td = (J*tj*tj + A*ta - J*tk*tk)/D;
	 }
      }
   }

   /**************************************************
    * If I previously swapped A & D, fix that now.
    **************************************************/
   if( swapAD )
   {
      double tmp;
      tmp = ta; ta = td; td = tmp;
      tmp = tj; tj = tk; tk = tmp;
      tmp =  A;  A =  D;  D = tmp;
   }

   /**************************************************
    * Adjust for negative moves as necessary
    **************************************************/
   if( negMove )
   {
      P  *= -1.0;
      J  *= -1.0;
   }

   init = true;
   return 0;
}

/***************************************************************************/
/**
Reset this object so it may be passed to an amplifier.  This will return an
error if the trajectory has not yet been calculated, or if it is currently 
being sent to another amp.

@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *TrjScurve::StartNew( void )
{
   if( !init ) return &ScurveError::NoCalc;
   if( inUse ) return &ScurveError::InUse;

   inUse = true;

   p = start;
   v = a = 0;
   j = J;
   remain = tj;
   seg = 0;

   return 0;
}

/***************************************************************************/
/**
Notify the trajectory object that it is no longer in use.
*/
/***************************************************************************/
void TrjScurve::Finish( void )
{
   inUse = false;
}

/***************************************************************************/
/**
Get the next PVT segment for this s-curve profile.
*/
/***************************************************************************/
const Error *TrjScurve::NextSegment( uunit &pout, uunit &vout, uint8 &tout )
{
   if( !inUse ) return &ScurveError::NotInUse;

   /**************************************************
    * We have previously calculate the position & 
    * velocity for this segment.  I'll set the output
    * values now.
    **************************************************/
#ifdef CML_ENABLE_USER_UNITS
   pout = p;
   vout = v;
#else
   if( p >= 0 ) pout = (uunit)(p + 0.5);
   else         pout = (uunit)(p - 0.5);

   if( v >= 0 ) vout = (uunit)(10.0 * v + 0.5);
   else         vout = (uunit)(10.0 * v - 0.5);
#endif

   if( seg == 7 )
   {
      tout = 0;
      return 0;
   }

   bool eos = false;
   double t;

   if( remain > 0.510 )
   {
      t = 0.255;
      tout = 255;
   }

   else if( remain > 0.254 )
   {
      tout = (uint8)floor(500*remain);
      t = 0.001 * tout;
   }

   else
   {
      eos = true;
      tout = (uint8)floor(remain*1000 + 0.01);
      t = remain;
   }

   p += v*t + a*t*t/2 + j*t*t*t/6;
   v += a*t + j*t*t/2;
   a += j*t;

   remain -= t;
   if( remain < 0 ) remain = 0;

   // We're done if that wasn't the end of a segment.
   if( !eos )
      return 0;

   // There's a good chance that the previous segment time 
   // wasn't an even number of milliseconds.  Find the amount
   // of time I went over the millisecond mark.
   double ms;
   double f = modf( 1000*t, &ms );
   double tf = 0;

   // Increment my output time if I didn't end on an even millisecond.
   if( f > 0.001 )
   {
      tf = 0.001 * (1.0 - f);
      tout++;
   }

   // Now, keep advancing to the next segment until I either come
   // to the end of the move, or find a segment with more then 1ms
   // available.
   while( AdvanceSegment(tf) && (remain < 0.001) );

   return 0;
}

/***************************************************************************/
/**
Move to the next s-curve segment with a non-zero time.  I also adjust my
local position, vel, etc values using the passed time.

@param tf The time value used to advance pos, vel, acc in the new segment.
@return zero if at the end of the move, else non-zero.
*/
/***************************************************************************/
int TrjScurve::AdvanceSegment( double &tf )
{
   // Find the next segment with a non-zero time
   switch( seg++ )
   {
      case 0: remain = ta; j =  0; if( remain > 0.000001 ) break; seg++;
      case 1: remain = tj; j = -J; break;
      case 2: remain = tv; j =  0; if( remain > 0.000001 ) break; seg++;
      case 3: remain = tk; j = -J; break;
      case 4: remain = td; j =  0; if( remain > 0.000001 ) break; seg++;
      case 5: remain = tk; j =  J; break;
      default: return 0;
   }

   // Advance the p,v,a values by the amount passed, or by
   // the total time in this segment if it's less.
   double t = (tf > remain) ? remain : tf;

   p += v*t + a*t*t/2 + j*t*t*t/6;
   v += a*t + j*t*t/2;
   a += j*t;

   // Reduce the total time left in this segment by the value used.
   remain -= t;
   tf -= t;
   return 1;
}

/***************************************************************************/
/**
Default constructor for multi-axis s-curve trajectory.
*/
/***************************************************************************/
LinkTrjScurve::LinkTrjScurve( void ){}

/***************************************************************************/
/**
Calculate a multi-axis s-curve trajectory.  This function calculates the 
straight line move between the two passed positions.
@param s The starting position
@param e The ending position
@param vel The max velocity
@param acc The max acceleration
@param dec The max deceleration
@param jrk The max jerk (rate of change of velocity)
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::Calculate( PointN &s, PointN &e, uunit vel, 
                                       uunit acc, uunit dec, uunit jrk )
{
   int d = s.getDim();

   CML_ASSERT( d <= CML_MAX_AMPS_PER_LINK );

   uunit dist = s.distance( e );

   const Error *err = trj.Calculate( dist, vel, acc, dec, jrk );
   if( err ) return err;

   double invDist = (dist!=0) ? 1.0/dist : 0.0;

   start.setDim( d );
   for( int i=0; i<d; i++ )
   {
      start[i] = s[i];
      scale[i] = (e[i] - s[i]) * invDist;
   }

   return 0;
}

/***************************************************************************/
/**
Start a new move using this trajectory.  The trajectory must have already
been calculated when this function is called.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::StartNew( void )
{
   return trj.StartNew();
}

/***************************************************************************/
/**
Retrieve the next segment of this trajectory.  The positions & velocities for
all axes are returned in the passed arrays.

@param pos An array which will be filled with position information.  
@param vel An array which will be filled with velocity information.
@param time A reference to a variable where the time (milliseconds) will be
       returned.
@return A pointer to an error object, or NULL on success.
*/
/***************************************************************************/
const Error *LinkTrjScurve::NextSegment( uunit pos[], uunit vel[], uint8 &time )
{
   uunit p, v;

   const Error *err = trj.NextSegment( p, v, time );
   if( err ) return err;

   for( int i=0; i<start.getDim(); i++ )
   {
      pos[i] = (uunit)(start[i] + p*scale[i]);
      vel[i] = (uunit)(v*scale[i]);
   }
   return 0;
}

/***************************************************************************/
/**
Finish this trajectory. 
*/
/***************************************************************************/
void LinkTrjScurve::Finish( void )
{
   trj.Finish();
}

#endif
