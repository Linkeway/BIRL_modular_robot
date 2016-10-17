/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML_Settings.h"
#if defined(CML_ALLOW_FLOATING_POINT) && defined(CML_ENABLE_USER_UNITS)

#include <math.h>
#include "CML.h"
#include "CML_Path.h"

CML_NAMESPACE_START()

// Error objects specific to path planning
CML_NEW_ERROR( PathError, BadVel,         "Illegal velocity value"         );
CML_NEW_ERROR( PathError, BadAcc,         "Illegal acceleration value"     );
CML_NEW_ERROR( PathError, VelNotInit,     "Velocity limit not yet set"     );
CML_NEW_ERROR( PathError, AccNotInit,     "Acceleration limit not yet set" );
CML_NEW_ERROR( PathError, BadPoint,       "The passed point doesn't match the path" );
CML_NEW_ERROR( PathError, Alloc,          "Unable to allocate memory for path" );
CML_NEW_ERROR( PathError, BadLength,      "An illegal negative length value was passed" );
CML_NEW_ERROR( PathError, Empty,          "Attempt to execute an empty path" );

// This constant defines the maximum angle (radians) between two line
// segments that I will accept without a full stop in between.
//
// FIXME: At the moment this is just set to 0.1 deg without much justification.
//        I should study the issue to come up with a better number
#define PI		3.14159265358979323846	/* pi */
#define PI_by_2		1.57079632679489661923	/* pi/2 */
#define MAX_ANGLE_ERROR      (0.1 * PI/180.0)

#define MIN_PVT_TIME         0.001

// local constant data
static const double jerkMult[] = {1,0,-1,0,-1,0,1};

/**
 Internal class used by Path planning code.
 */
class PathElement
{
protected:
   PathElement *next;
   PathElement *prev;

   // These values are calculated when a segment is first 
   // initialized and never change during the life of the 
   // segment.
   double length;   // Length of current segment

   double velMax;   // Maximum velocity limit during this segment
   double accMax;   // Acceleration limit to use during this segment
   double decMax;   // Deceleration limit to use during this segment (use accel limit if <= 0)
   double jrkMax;   // Jerk limit to use during this segment (no limit if <= 0)

   // These parameters will change as more segments are added
   // to the path.
   double velEnd;   // Maximum velocity at end of segment
   double velPeak;  // Maximum ending velocity possible based on earlier segments

   // Once I've determined the starting and ending velocity for a segment, I'll 
   // calculate the times of each of the seven possible sub-segments in the move.
   bool calculated;
   double SegT[7], SegP[7], SegV[7], SegA[7];

   /**
    * Set the segment length.  This must be called in the 
    * constructor of the segment object before the segment
    * is added to a path.
    */
   void setLength( double L )
   {
      length = L;
   }

public:
   PathElement( void )
   {
      velPeak = velEnd = 0.0;
      velMax = accMax = decMax = jrkMax = 0.0;
      length = 0.0;
      next = prev = 0;
      calculated = false;

      for( int i=0; i<7; i++ )
	 SegT[i] = SegP[i] = SegV[i] = SegA[i] = 0;
   }

   virtual ~PathElement()
   {
      Unlink();
   }

   void Unlink( void )
   {
      if( prev ) prev->next = next;
      if( next ) next->prev = prev;
      next = prev = 0;
   }

   /**
    * Add this segment to the end of the 
    * passed path.
    */
   void Add( PathElement *pe )
   {
      // Add this segment after the passed one.
      if( pe ) pe->next = this;
      this->prev = pe;

      // Find the peak velocity that could be 
      // reached at the end of this segment if
      // I didn't have to worry about stopping in 
      // the future.
      //
      // This is the previous segment's peak velocity
      // plus the increase I could provide based on 
      // this segment's acceleration & length.
      double Vstart = 0;
      if( pe ) Vstart = pe->velPeak;
      velPeak = getMaxVelInc( Vstart, getMaxAcc() );
   }

   PathElement *getPrev( void ){ return prev; }
   PathElement *getNext( void ){ return next; }

   double getVelStart( void )
   {
      PathElement *pe = getPrev();
      return (pe) ? pe->getVelEnd() : 0.0;
   }

   double getVelEnd( void )
   {
      return velEnd;
   }

   /**
    * Initialize some internal parameters based on 
    * segment length and various limits (velocity,
    * accel, decel).
    */
   void Init( double V, double A, double D, double J )
   {
      // Save the constraints
      velMax = V; 
      accMax = A;
      decMax = D;
      jrkMax = J;
   }

   /// Return the length of the path element
   virtual double getLength( void ){ return length; }

   /// Get the maximum velocity limit used during this 
   /// segment.
   virtual double getMaxVel( void ){ return velMax; }

   /// Get the acceleration limit used during this segment.
   virtual double getMaxAcc( void ){ return accMax; }

   /// Get the deceleration limit used during this segment
   virtual double getMaxDec( void )
   {
      return (decMax <= 0.0) ? accMax : decMax;
   }

   /// Get the jerk limit used during this segment.  
   /// Note that a value of <= 0.0 means no jerk limit
   virtual double getMaxJrk( void ){ return jrkMax; }

   /// Return true if jerk limits are being used
   virtual bool usingJerkLimits( void ){ return getMaxJrk() >= 0.0; }

   // Return the maximum starting velocity that this segment
   // could possibly handle.  This is just the end velocity 
   // increased by the amount we could speed up going back 
   // through this segment.
   virtual double getMaxStartVel( void )
   {
      return getMaxVelInc( velEnd, getMaxDec() );
   }

   // Return the maximum velocity increase possible for this segment
   // based on the length of the segment and it's limits.
   // We assume some known starting velocity.
   //
   // @param Vs starting velocity to use
   // @param A Acceleration limit to use
   // @return Maximum ending velocity
   virtual double getMaxVelInc( double Vs, double A )
   {
      double P = getLength();
      double Vmax = getMaxVel();

      // If we aren't using jerk limiting, then this is 
      // simply the starting velocity plus the amount that we
      // can accelerate during this segment.
      if( !usingJerkLimits() )
      {
	 double v = sqrt( Vs*Vs + 2*A*P );
	 if( v > Vmax ) v = Vmax;
	 return v;
      }

      // If we are using jerk limits, this is considerably more complex.
      // Note that we are assuming that acceleration is zero between 
      // segments to keep things from getting too hairy.
      //
      // First, see what our max veloctiy would be if we only used
      // the jerk and position limits.
      //
      // P = 2*Vs*t + J*t^3
      //   Vs = starting velocity
      //   P  = total segment length
      //   J  = jerk limit
      //   t  = half time for segment
      //
      // solve this for t:
      double J = getMaxJrk();
      double M = pow( sqrt( (32*Vs*Vs*Vs+27*J*P*P)/(108*J*J*J) )+P/(2*J), 1.0/3.0);
      double t = M -(2*Vs)/(3*J*M);

      // The peak acceleration would be at time t.  Make sure this doesn't
      // exceed my limit.
      if( J*t <= A )
      {
	 // OK, we didn't exceed the accel limit, so find the max ending
	 // velocity.  If this is over my maximum then just return the max.
	 double v = Vs + J * t * t;
	 if( v > Vmax ) v = Vmax;
	 return v;
      }

      // We exceeded the accel limit, so I'll recalculate using that limit
      // as well as my jerk limit.  Basically, I'll split the segment into
      // three parts, two parts will run at the jerk limits and one will
      // run at the accel limit

      // Find the time it will take to run at the jerk limit
      double tj = A/J;

      // Find the time at the accel limit
      double AA = A*A;
      double AAAA = AA*AA;
      double JJ = J*J;
      double ta = (sqrt(8*A*JJ*P + 4*Vs*Vs*JJ - 4*Vs*AA*J + AAAA) - 2*Vs*J - 3*AA)/(2*A*J);

      // Find the velocity at the end of those times
      double v = Vs + A*(tj+ta);

      // Limit based on our max
      if( v > Vmax ) v = Vmax;
      return v;
   }

   /// Adjust the ending velocity and acceleration limits for 
   /// this segment based on the starting values of the next
   /// segment.
   ///
   /// This is called as new segments are added after this one.
   ///
   /// @return true if the ending velocity changed.
   bool adjustEndState( void )
   {
      // If my running times have already been calculated, then
      // my ending velocity can no longer be adjusted.  This can 
      // happen if segments are being added to a running trajectory.
      if( calculated )
	 return false;

      // Get the starting velocity of the segment immediately following
      // this one.  There should always be a segment following this one.
      PathElement *e = getNext();
      CML_ASSERT( e );

      double v = (e==0) ? 0 : e->getMaxStartVel();

      // Limit the ending velocity to my local peak
      if( v > velPeak ) v = velPeak;

      // If the new ending velocity is greater then my current
      // value, then adjust it and return true.
      if( v > velEnd )
      {
	 velEnd = v;
	 return true;
      }

      // If the new ending velocity isn't any greater then my current
      // one, then I now have enough information to calculate my running
      // times.
      Calculate();
      return false;
   }

   // Calculate run times with no jerk limits.  This is quicker & simpler
   // then the full blown calculations that include jerk limits.
   virtual void CalcNoJrk( void )
   {
      double ve = velEnd;
      double vs = getVelStart();
      double P = length;
      double A = getMaxAcc();
      double D = getMaxDec();
      double V = getMaxVel();

      // Assume for the moment that we will hit our accel & decel limits.
      double ta = (V-vs) / A;
      double td = (V-ve) / D;
      double tv;
      double remain = P - (vs*ta + ta*ta*A/2) - (ve*td + td*td*D/2);

      if( remain >= 0 )
	 tv = remain / V;

      else
      {
	 ta = sqrt( (A+D)*(D*vs*vs + A*ve*ve + 2*A*D*P) ) / (A*(A+D)) - vs/A;
	 td = (A*ta+vs-ve)/D;
	 tv = 0.0;
      }

      // Set the times for each sub-segment
      SegT[0] = 0.0;
      SegT[1] = ta;
      SegT[2] = 0.0;
      SegT[3] = tv;
      SegT[4] = 0.0;
      SegT[5] = td;
      SegT[6] = 0.0;

      // Set the acceleration at the end of each sub-segment
      SegA[0] = A;
      SegA[1] = 0;
      SegA[2] = 0;
      SegA[3] = 0;
      SegA[4] = -D;
      SegA[5] = 0;
      SegA[6] = 0;

      // Fill in the position, and velocity for 
      // the end of each of the sub-segments.
      double p = 0;
      double v = getVelStart();

      SegP[0] = 0;
      SegV[0] = v;
      for( int i=1; i<7; i+=2 )
      {
	 double t = SegT[i];
	 double a = SegA[i-1];

	 p += v*t + a*t*t/2;
	 v += a*t;

	 SegP[i] = SegP[i+1] = p;
	 SegV[i] = SegV[i+1] = v;
      }

      return;
   }

   bool CalcForVel( double V, bool force=false )
   {
      double Ve = velEnd;
      double Vs = getVelStart();
      double P = length;
      double A = getMaxAcc();
      double D = getMaxDec();
      double J = getMaxJrk();

      // We start out assuming that we will hit the max velocity.
      // Find the times required in jerk and accel segments
      // Also, find the distance moved getting up to velocity.
      double tj, ta, Pup;

      if( J * (V-Vs) < A*A )
      {
	 ta = 0;
	 tj = sqrt( (V-Vs)/J );
	 Pup = J*tj*tj*tj + 2*Vs*tj;
      }
      else
      {
	 tj = A/J;
	 ta = (V-Vs)/A - tj;
	 Pup = Vs*(2*tj+ta) + J*tj*tj*tj + 3*J/2*ta*tj*tj + J/2*ta*ta*tj;
      }

      // If I'm already past my position limit, then quit now
      if( Pup > P ) return false;

      // Same thing for the deceleration portion of the segment
      double tk, td, Pdn;

      if( J * (V-Ve) < D*D )
      {
	 td = 0;
	 tk = sqrt( (V-Ve)/J );
	 Pdn = J*tk*tk*tk + 2*Ve*tk;
      }
      else
      {
	 tk = D/J;
	 td = (V-Ve)/D - tk;
	 Pdn = Ve*(2*tk+td) + J*tk*tk*tk + 3*J/2*td*tk*tk + J/2*td*td*tk;
      }

      // If the sum of these two distances exceeds my total length, 
      // then I can't hit this velocity during my move.
      //
      // Note that I add a small amount of margin to deal with floating
      // point round-off error in some specific cases
      double Ptest = force ? P+0.01 : P;

      if( Pup+Pdn > Ptest )
	 return false;

      // Record the move times
      SegT[0] = tj;
      SegT[1] = ta;
      SegT[2] = tj;
      SegT[3] = (P-Pup-Pdn)/V;
      SegT[4] = tk;
      SegT[5] = td;
      SegT[6] = tk;

      if( SegT[3] < 0 ) SegT[3] = 0;
      return true;
   }

   // Do the main part of my calculations.  This function fills in the array
   // of times in each of the 7 possible sub-segments.
   void CalcTimes( void )
   {
      double Ve = velEnd;
      double Vs = getVelStart();
      double P = length;
      double A = getMaxAcc();
      double D = getMaxDec();
      double V = getMaxVel();
      double J = getMaxJrk();

      // We start out assuming that we will hit the max velocity.
      // If this calculation is successful, then we're done
      if( CalcForVel( V ) )
	 return;

      // Make a quick check here for a zero length segment.  
      if( P <= 0.0 )
      {
	 for( int i=0; i<7; i++ ) SegT[i] = 0;
	 return;
      }

      // OK, we aren't going to hit the velocity limit in this move.  
      // I'll try running at the accel & decel limits only
      double ta = ( sqrt( 8*A*D*J*J*P*(A+D) + 
	                  4*J*J*(Vs*Vs*D*D + (Vs*Vs+Ve*Ve)*A*D + Ve*Ve*A*A) -
		          4*A*D*J*(Ve*D*D + (Vs+Ve)*A*D + Vs*A*A) + 
		          A*A*D*D*( D*D + 2*A*D + A*A )
	                ) 
	            -2*J*Vs*(A+D) -A*D*D -3*A*A*D - 2*A*A*A
	          ) 
	          / ( 2*A*J*(A+D) );

      double tj = A/J;
      double tk = D/J;
      double td = (Vs - Ve + J*tj*ta + J*tj*tj - J*tk*tk)/(J*tk);

      // If both of these times came out positive, we're done
      if( (ta >= 0.0) && (td >= 0.0) )
      {
	 SegT[0] = tj;
	 SegT[1] = ta;
	 SegT[2] = tj;
	 SegT[3] = 0;
	 SegT[4] = tk;
	 SegT[5] = td;
	 SegT[6] = tk;
	 return;
      }

      // We can't reach both the accel & decel limits.
      // There isn't a simple formulat for calculating out the optimal times
      // in this case, so I'll itterate with various maximum velocities until
      // I find one that will work.
      double Vup, Vdn;

      // I'll find the max velocity that I'd use without jerk limiting as an
      // initial upper limit
      CalcNoJrk();
      Vup = SegV[1];

      // For my lower limit, I'll pick the higher of the starting or ending velocity
      Vdn = (Vs>Ve) ? Vs : Ve;

      // First calculation uses the lower velocity limit.  
      // This should never fail.
      CalcForVel( Vdn, true );

      // Itterate as many as 10 times to try to get a faster move.
      // I'll quit when my constant velocity segment is less then
      // 1 millisecond long.
      for( int i=0; (i<10) && (SegT[3] > MIN_PVT_TIME); i++ )
      {
	 V = (Vup+Vdn)/2;
	 if( CalcForVel( V ) )
	    Vdn = V;
	 else
	    Vup = V;
      }

      return;
   }

   // Calculate the running times used by this segment.  The running times
   // define the velocity profile of the segment.  They are calculated when
   // starting and ending states of the segment have been found.
   virtual bool Calculate( void )
   {
      if( calculated ) return true;
      calculated = true;

      // If not using jerk limits, use simpler calculations
      if( !usingJerkLimits() )
      {
	 CalcNoJrk();
	 return false;
      }

      // Calculate the sub-segment times
      CalcTimes();

      // Fill in the position, velocity and accel for 
      // the end of each of the sub-segments.
      double p = 0;
      double v = getVelStart();
      double a = 0;
      double Jmax = getMaxJrk();

      for( int i=0; i<7; i++ )
      {
	 double J = Jmax * jerkMult[i];
	 double t = SegT[i];

	 p += v*t + a*t*t/2 + J*t*t*t/6;
	 v += a*t + J*t*t/2;
	 a += J*t;

	 SegP[i] = p;
	 SegV[i] = v;
	 SegA[i] = a;
      }
      return false;
   }

   /**
     Return the total amount of time it will take me to run through this segment.
     @return time in seconds.
    */
   virtual double getDuration( void )
   {
      Calculate();

      double t = 0;
      for( int i=0; i<7; i++ )
	 t += SegT[i];
      return t;
   }

   /**
    * Give a current time into this segment (less then the 
    * segment duration), return the next largest time into the
    * segment with a fixed acceleration.
    *
    * @param t The starting time into the segment is passed in.
    *          The new time value is passed out.
    * @return true if a new time within the segment could be found
    */
   virtual bool getNextSegTime( double &t )
   {
      Calculate();
      t += MIN_PVT_TIME;

      double x = 0;
      for( int i=0; i<7; i++ )
      {
	 x += SegT[i];
	 if( t < x )
	 {
	    t = x;
	    return false;
	 }
      }
      t = x;
      return true;
   }

   /**
     Get the position and velocity along this segment's path at 
     the specified time into the segment.
     @param t The time (seconds) into the segment.  Must not be
              greater then the segment's duration
     @param pos The position along the path is returned here.
     @param vel The velocity along the path is returned here.
    */
   virtual void getPathPos( double t, double &pos, double &vel )
   {
      Calculate();

      double p = 0;
      double v = getVelStart();
      double a = 0;

      for( int i=0; i<7; i++ )
      {
	 if( t > SegT[i] )
	 {
	    t -= SegT[i];
	    p = SegP[i];
	    v = SegV[i];
	    a = SegA[i];
	 }
	 else
	 {
	    double J = getMaxJrk();
	    if( J < 0 ) J = 0;
	    J *= jerkMult[i];

	    pos = p + v*t + a*t*t/2 + J*t*t*t/6;
	    vel = v + a*t + J*t*t/2;
	    return;
	 }
      }
   }

   virtual const Error *getTrjSeg( double t, uunit pos[], uunit vel[] ) = 0;
};

/**
 * Line segment path element.
 */
class LineSeg: public PathElement
{
   Point<PATH_MAX_DIMENSIONS> start;
   double direction, sinDir, cosDir;
public:

   LineSeg( PointN &start, double dir, double len )
   {
      this->start = start;
      direction = dir;
      setLength( len );

      sinDir = sin(direction);
      cosDir = cos(direction);
   }

   const Error *getTrjSeg( double t, uunit p[], uunit v[] )
   {
      double pos, vel;

      getPathPos( t, pos, vel );

      p[0] = start[0] + cosDir * pos;
      v[0] = cosDir * vel;

      if( start.getDim() > 1 )
      {
	 p[1] = start[1] + sinDir * pos;
	 v[1] = sinDir * vel;
      }
      return 0;
   }
};

class ArcSeg: public PathElement
{
   Point<PATH_MAX_DIMENSIONS> center;
   double radius, startAng, totAng;
   double velCentrip;
public:
   ArcSeg( PointN &ctr, double r, double start, double tot )
   {
      center = ctr;
      radius = r;
      startAng = start;
      totAng = tot;
      setLength( radius * fabs(tot) );
      velCentrip = -1;
   }

   // Limit max acceleration around a curve based on 
   // centripetal acceleration.
   double getMaxVel( void )
   {
      // Calculate a velocity limit based on centripetal accel
      if( velCentrip <= 0 )
      {
	 double A = getMaxAcc();
	 double D = getMaxDec();
	 if( D < A ) A = D;

	 velCentrip = sqrt( A * radius );
      }

      double max = PathElement::getMaxVel();
      if( velCentrip < max )
	 return velCentrip;
      return max;
   }


   const Error *getTrjSeg( double t, uunit p[], uunit v[] )
   {
      double pos, vel;

      getPathPos( t, pos, vel );

      pos /= radius;

      double ang;
      if( totAng < 0 )
      {
	 vel *= -1.0;
	 ang = startAng + pos;
      }
      else  
	 ang = startAng - pos;

      double sinAng = sin(ang);
      double cosAng = cos(ang);

      p[0] = center[0] + cosAng * radius;
      p[1] = center[1] + sinAng * radius;
      v[0] =  vel * sinAng;
      v[1] = -vel * cosAng;

      return 0;
   }

   bool getNextSegTime( double &t )
   {
      double oldT = t;
      bool ret = PathElement::getNextSegTime(t);

      // Force arc updates to happen at least every 10ms.
      // I should optimize this based on radius and speed.
      if( t-oldT > 0.01 )
      {
	 t = oldT + 0.01;
	 return false;
      }
      return ret;
   }
};

/**
 * Path segment used to delay for a specified amount of time.
 */
class DelaySeg: public PathElement
{
   Point<PATH_MAX_DIMENSIONS> pos;
   double delayTime;
public:
   DelaySeg( PointN &p, double t )
   {
      pos = p;
      delayTime = t;
      setLength( 0.0 );
   }
   virtual double getMaxVel( void ){ return 0.0; }
   virtual bool Calculate( void )
   {
      if( calculated ) return true;
      calculated = true;
      SegT[3] = delayTime;
      return false;
   }

   const Error *getTrjSeg( double t, uunit p[], uunit v[] )
   {
      for( int i=0; i<pos.getDim(); i++ )
      {
	 p[i] = pos[i];
	 v[i] = 0;
      }
      return 0;
   }
};

Path::Path( uint d )
{
   CML_ASSERT( d <= PATH_MAX_DIMENSIONS );

   if( d > PATH_MAX_DIMENSIONS )
      d = PATH_MAX_DIMENSIONS;
   dim = d;
   maxVel = 0.0;
   maxAcc = 0.0;
   maxDec = -1.0;
   maxJrk = -1.0;
   first = last = 0;
   dirEnd = 0.0;
   posEnd.setDim(d);
   posStart.setDim(d);
   Reset();
}

Path::~Path( void )
{
   mtx.Lock();
   while( first )
   {
      PathElement *pe = first;
      first = pe->getNext();
      delete pe;
   }
   mtx.Unlock();
}

void Path::Reset( void )
{
   mtx.Lock();
   crntSeg = first;
   mtx.Unlock();
   segTime = 0;
}

const Error *Path::SetStartPos( PointN &p )
{
   CML_ASSERT( p.getDim() == GetDim() );
   if( p.getDim() != GetDim() )
      return &PathError::BadPoint;

   posStart = p;
   return 0;
}


/**
  Offset the passed array of relative positions based
  on the starting position set by the user.
  @p Array of positions to be adjusted
 */
void Path::OffsetPos( double p[] )
{
   for( int i=0; i<dim; i++ )
      p[i] += posStart[i];
}

/** 
 * Set a new velocity limit which will apply to any new
 * segments added to the path.
 *
 * The passed velocity must be >= zero.
 *
 * @param v The velocity limit to use
 * @return An error object pointer or NULL on success
 */
const Error *Path::SetVel( uunit v )
{
   if( v <= 0.0 ) return &PathError::BadVel;
   maxVel = v;
   return 0;
}

/** 
 * Set a new acceleration limit which will apply to any new
 * segments added to the path.
 *
 * The passed acceleration value must be >= zero.
 *
 * @param a The acceleration limit to use
 * @return An error object pointer or NULL on success
 */
const Error *Path::SetAcc( uunit a )
{
   if( a <= 0.0 ) return &PathError::BadAcc;
   maxAcc = a;
   return 0;
}

/** 
 * Set a new deceleration limit which will apply to any new
 * segments added to the path.
 *
 * If a positive deceleration is passed, then this will be used
 * as the deceleration limit.  If a zero or negative value is passed
 * then the acceleration value will be used for deceleration as well
 * as acceleration.
 *
 * @param d The deceleration limit to use
 * @return An error object pointer or NULL on success
 */
const Error *Path::SetDec( uunit d )
{
   maxDec = d;
   return 0;
}

/** 
 * Set a new jerk limit which will apply to any new
 * segments added to the path.
 *
 * Setting a jerk of <= 0 will disable jerk limiting.
 *
 * @param j The jerk limit to use
 * @return An error object pointer or NULL on success
 */
const Error *Path::SetJrk( uunit j )
{
   maxJrk = j;
   return 0;
}

/**
 * Add a new segment to the end of this path.
 */
const Error *Path::AddSegment( PathElement *e )
{
   // Check for uninitialized velocity and accelerations
   if( maxVel <= 0.0 ) return &PathError::VelNotInit;
   if( maxAcc <= 0.0 ) return &PathError::AccNotInit;

   // Initialize the new segment
   e->Init( maxVel, maxAcc, maxDec, maxJrk );

   // Add this segment to the end of my path
   mtx.Lock();
   e->Add( last );
   last = e;
   if( !first )
   {
      first = e;
      crntSeg = e;
      segTime = 0;
   }
   mtx.Unlock();

   // Adjust the ending velocities of preceeding 
   // segments.
   while( 1 )
   {
      e = e->getPrev();
      if( !e ) break;

      if( !e->adjustEndState() )
	 break;
   }

   // Handle internal calculations for all segments
   // that have been fully added.
   while( e )
   {
      if( e->Calculate() )
	 break;
      e = e->getPrev();
   }

   return 0;
}

const Error *Path::AddLine( PointN &p )
{
   // Make sure the passed point is of the correct dimension
   CML_ASSERT( p.getDim() == GetDim() );
   if( p.getDim() != GetDim() )
      return &PathError::BadPoint;

   // Adjust the passed point to find a position relative to 
   // the starting position.
   p -= posStart;

   // Find the direction of travel between the current position
   // and the passed point.
   double dx = p[0] - posEnd[0];
   double dy = 0.0;

   if( GetDim() > 1 )
      dy = p[1] - posEnd[1];

   double dirMove = atan2( dy, dx );

   // If the move direction isn't in line with my current direction,
   // then I'll have to come to a halt before adding the line segment.
   // Otherwise, I would have an infinite acceleration during the 
   // direction change.
   if( fabs(dirMove - dirEnd) > MAX_ANGLE_ERROR )
      Pause(0);

   double len = posEnd.distance( p );

   // Now, add the line segment to my path
   LineSeg *seg = new LineSeg( posEnd, dirMove, len );
   if( !seg )
      return &PathError::Alloc;

   const Error *err = AddSegment( seg );
   if( err ) return err;

   // Update my ending position & direction
   posEnd = p;
   dirEnd = dirMove;
   return 0;
}

const Error *Path::AddLine( uunit length )
{
   // Length must be a positive value.
   CML_ASSERT( length >= 0 );

   // Calculate the ending position based on the
   // current position and direction of travel
   Point<PATH_MAX_DIMENSIONS> p;
   p.setDim( GetDim() );

   p = posEnd;
   p[0] += cos(dirEnd) * length;
   if( GetDim() > 1 )
      p[1] += sin(dirEnd) * length;

   // add the line segment to my path
   LineSeg *seg = new LineSeg( posEnd, dirEnd, length );
   if( !seg ) return &PathError::Alloc;

   const Error *err = AddSegment( seg );
   if( err ) return err;

   // Update my ending position & direction
   posEnd = p;
   return 0;
}

const Error *Path::AddArc( PointN &center, double angle )
{
   // Can't add an arc to a one dimensional path
   CML_ASSERT( GetDim() > 1 );

   // Make sure the center position has the right number of dimensions
   CML_ASSERT( GetDim() == center.getDim() );

   // Adjust the center to find a position relative to 
   // the starting position.
   center -= posStart;

   // Find the starting angle on the arc
   double deltaX = center[0] - posEnd[0];
   double deltaY = center[1] - posEnd[1];
   double startAng = PI + atan2( deltaY, deltaX );

   // See if there will be an abrupt change in direction when we start
   // this path.  If so, then I'll need to come to a halt first.
   double deltaDir;
   if( angle < 0 )
      deltaDir = startAng - dirEnd + PI_by_2;
   else
      deltaDir = startAng - dirEnd - PI_by_2;

   if( fabs(deltaDir) > MAX_ANGLE_ERROR )
      Pause(0);

   double radius = center.distance( posEnd );
   ArcSeg *seg = new ArcSeg( center, radius, startAng, angle );
   if( !seg ) return &PathError::Alloc;

   const Error *err = AddSegment( seg );
   if( err ) return err;

   // Update the ending position and direction
   double ang = startAng - angle;
   posEnd[0] = center[0] + cos(ang) * radius;
   posEnd[1] = center[1] + sin(ang) * radius;

   if( angle < 0 ) 
      dirEnd = ang + PI_by_2;
   else 
      dirEnd = ang - PI_by_2;

   return 0;
}

const Error *Path::AddArc( double radius, double angle )
{
   // radius must be a positive value.
   CML_ASSERT( radius >= 0 );

   // Can't add an arc to a one dimensional path
   CML_ASSERT( GetDim() > 1 );

   // Find the center.  This will be a point on the line that passes
   // through the current end position and is orthogonal to the current
   // direction of motion.
   Point<PATH_MAX_DIMENSIONS> center;
   center.setDim( GetDim() );
   center = posEnd;

   double startAng;

   // Positive angles are clockwise rotation
   if( angle < 0 )
   {
      center[0] -= radius * sin(dirEnd);
      center[1] += radius * cos(dirEnd);
      startAng = dirEnd - PI_by_2;
   }
   else
   {
      center[0] += radius * sin(dirEnd);
      center[1] -= radius * cos(dirEnd);
      startAng = dirEnd + PI_by_2;
   }

   ArcSeg *seg = new ArcSeg( center, radius, startAng, angle );
   if( !seg ) return &PathError::Alloc;

   const Error *err = AddSegment( seg );
   if( err ) return err;

   // Update the ending position and direction
   double ang = startAng - angle;
   posEnd[0] = center[0] + cos(ang) * radius;
   posEnd[1] = center[1] + sin(ang) * radius;

   dirEnd -= angle;
   return 0;
}

const Error *Path::Pause( double sec )
{
   DelaySeg *seg = new DelaySeg( posEnd, sec );
   if( !seg ) return &PathError::Alloc;
   return AddSegment( seg );
}

int Path::GetDim( void )
{
   return dim;
}

uint8 Path::GetTime( void )
{
   // Ask the current segment for the next largest time
   // increment it can provide.
   double oldT = segTime;
   uint ms;

   while( 1 )
   {
      bool end = crntSeg->getNextSegTime( segTime );
      double diff = segTime - oldT;

      // My maximum PVT segment length is 0.255 seconds.  If the time 
      // difference is more then abount 500ms then I'll use this max
      if( diff >= 0.500 )
      {
	 ms = 255;
	 break;
      }

      // If the time segment is greater then 255ms, I'll use half of it.
      if( diff >= 0.255 )
      {
	 ms = (uint8)(diff * 500);
	 break;
      }

      // If the time is at least my minimum, I'll use it.
      if( diff >= MIN_PVT_TIME )
      {
	 ms = (uint8)(diff * 1000);
	 break;
      }

      // If I'm not at the end of this segment, just ask for 
      // a larger time increment
      if( !end ) continue;

      // If this is not the last segment of the trajectory, then move
      // to the next segment.
      if( crntSeg->getNext() )
      {
	 oldT -= segTime;
	 segTime = 0.0;
	 crntSeg = crntSeg->getNext();
	 continue;
      }

      // I've found the end of the profile.  If there is at least a 
      // millisecond of time available then I'll use that.  Otherwise 
      // I'll return zero.
      if( diff >= 0.001 )
      {
	 ms = (uint8)(1000*diff);

	 break;
      }

      return 0;
   }

   // At this point, ms give the time in milliseconds that I'll use for 
   // this segment.  I need to adjust the segment time to reflect that.
   segTime = oldT + 0.001 * ms;
   return ms;
}

const Error *Path::StartNew( void )
{
   Reset();
   return (crntSeg) ? 0 : &PathError::Empty;
}

const Error *Path::NextSegment( uunit pos[], uunit vel[], uint8 &time )
{
   if( !crntSeg ) return &PathError::Empty;

   const Error *err = crntSeg->getTrjSeg( segTime, pos, vel );
   time = GetTime();

   OffsetPos( pos );

   return err;
}

bool Path::PlayPath( double timeInc, double pos[], double vel[] )
{
   if( !crntSeg ) return true;

   crntSeg->getTrjSeg( segTime, pos, vel );
   segTime += timeInc;

   OffsetPos( pos );

   double t = crntSeg->getDuration();
   if( segTime >= t )
   {
      segTime -= t;
      crntSeg = crntSeg->getNext();
   }

   return (crntSeg) ? false : true;
}

CML_NAMESPACE_END()

#endif

