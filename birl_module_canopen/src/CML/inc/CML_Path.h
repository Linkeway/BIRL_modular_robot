/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#ifndef _DEF_INC_PATH
#define _DEF_INC_PATH

#include "CML_Threads.h"
#include "CML_Trajectory.h"

CML_NAMESPACE_START()

/**
  This class represents errors returned by the path Path object.
 */
class PathError: public Error
{
public:
   static const PathError BadVel;         ///< Illegal velocity value
   static const PathError BadAcc;         ///< Illegal acceleration value
   static const PathError VelNotInit;     ///< Velocity limit not yet set
   static const PathError AccNotInit;     ///< Acceleration limit not yet set
   static const PathError BadPoint;       ///< The passed point doesn't match the path
   static const PathError Alloc;          ///< Unable to allocate memory for path 
   static const PathError BadLength;      ///< An illegal negative length value was passed
   static const PathError Empty;          ///< Attempt to execute an empty path

protected:
   /// Standard protected constructor
   PathError( uint16 id, const char *desc ): Error( id, desc ){}
};

#define PATH_MAX_DIMENSIONS        2

class PathElement;

/**
  Multi-axis complex trajectory path.
  The object may be used to construct one or two dimensional trajectories 
  built out of line segments and arcs.
 */
class Path : public LinkTrajectory 
{
   // Dimension (number of axes) of the path.
   int dim;       

   // Position at end of most recent segment added
   Point<PATH_MAX_DIMENSIONS> posEnd;

   // Starting position of the trajectory
   Point<PATH_MAX_DIMENSIONS> posStart;

   // Direction of motion at end of last segment
   double dirEnd;

   /// Private copy constructor (not supported)
   Path( const Path& );

   /// Private assignment operator (not supported)
   Path& operator=( const Path& );

protected:
   /*
    * Global limits.  These limits apply to new
    * segments added to the path.
    */
   double maxVel;
   double maxAcc;
   double maxDec;
   double maxJrk;

   // Mutex used to protect access to some internal data
   Mutex mtx;

   // Linked list of path elements
   PathElement *first, *last;

   // Current path element while running through path
   PathElement *crntSeg;

   // Time into current segment of next point to retrieve
   double segTime;

   const Error *AddSegment( PathElement *e );
   uint8 GetTime( void );

   void OffsetPos( double p[] );

public:

   /**
     Path object constructor.  The number of dimensions for the path
     must be passed.  This object currently supports one and two 
     dimensional (i.e. one and two axis) path construction.
     @param d The number of dimensions for the path.  Must be either
              one or two for now.
    */
   Path( uint d );

   /**
     Destructor for the path object.
    */
   virtual ~Path( void );

   /**
     Reset the path to the first position.
     This should be called before the path is passed to the Linkage object
     as a trajectory to run.
    */
   virtual void Reset( void );

   /**
     Set the initial position for the path.  This method may be used to 
     start a path at a position other then (0,0) which is the default if
     no staring position is set.  

     The starting position may be set at any time, either before or after 
     adding segments to the path.  Internally, the segments are all stored 
     as relative positions.

     @param p The starting position for this path.
     @return An error object or null on success
    */
   virtual const Error *SetStartPos( PointN &p );

   /**
     Set the velocity limit for the current location.
     Velocity limits must be greater then zero.
     @param v The maximum velocity (position units / second)
     @return An error object or null on success
    */
   virtual const Error *SetVel( uunit v );

   /** 
     Set the acceleration limit for the current location.
     Acceleration limits must be greater then zero.
     @param a The maximum acceleration (position units / second / second)
     @return An error object or null on success
    */
   virtual const Error *SetAcc( uunit a );

   /**
     Set the deceleration limit for the current location.
     Note that setting the deceleration limit less then or
     equal to zero will cause the acceleration value
     to be used for deceleration also.
     @param d The maximum deceleration (position units / second / second)
     @return An error object or null on success
    */
   virtual const Error *SetDec( uunit d );

   /**
     Set the jerk limit for the current location.
     Note that setting the jerk limit to a value less then or
     equal to zero will cause the path to be calculated with 
     no jerk limiting.  
     @param j The jerk limit (position units / second / second / second)
     @return An error object or null on success.
   */
   virtual const Error *SetJrk( uunit j );

   /**
     Add a line segment from the current position to the 
     specified point.  The direction of motion required 
     to move from the current position to the given
     point will be compared to the direction of motion at
     the end of the last segment.  If these directions 
     change then the addition of this new point will require
     an abrupt change of direction.  In this case, the 
     initial velocity will be set to zero.

     @param p The point to move to.
     @return An error object or null on success
    */
   virtual const Error *AddLine( PointN &p );

   /**
     Add a line segment of the specified length.  The direction
     of motion will remain the same as it was at the end of the 
     last added segment.  If this is the first segment added to 
     the path, then the direction will be positive motion in the
     first axis.

     @param length The length of the line segment to add.
     @return An error object or null on success
    */
   virtual const Error *AddLine( uunit length );

   /**
     Add an arc with the specified radius and angle (radians).
     The arc will start at the current position and will move in either a 
     clockwise (positive angle), or counter-clockwise (negative angle)
     direction.

     @param radius The radius of the arc
     @param angle The number of radians to rotate through.  Positive 
                 values will result in clockwise rotation.

     @return An error object or null on success
    */
   virtual const Error *AddArc( double radius, double angle );

   /**
     Add an arc with the specified center point and angle (radians).
     The arc will start at the current position and will move in 
     clockwise (positive angle), or counter-clockwise (negative angle)
     direction.

     @param center The center point of the arc.
     @param angle The number of radians to rotate through.  Positive 
                  values will result in clockwise rotation.

     @return An error object or null on success
    */
   virtual const Error *AddArc( PointN &center, double angle );

   /**
     Set the current velocity to 0 and pause for the specified 
     amount of time.

     @param sec The time to pause (must be >= 0).  Time is specified
                in seconds.
    */
   virtual const Error *Pause( double sec );

   /**
     Get the dimension (i.e. number of axes) of the path.
     @return The path dimension
    */
   virtual int GetDim( void );

   /**
     Get the next trajectory segment.  This method is called by the Linkage object
     when as it passes the trajectory informatoin up to the amplifiers.
   
      @param pos An array where the position values will be
             returned.  This array will be at least D elements
             long, where D is the trajectory dimension as
             returned by LinkTrajectory::GetDim()
     
      @param vel An array where the velocity values will be
             returned.
     
      @param time The segment time is returned here.  This is
             in milliseconds and ranges from 1 to 255.  If
             zero is returned, this is the last frame in the profile.
     
      @return A pointer to an error object on failure, or NULL on success.
    */
   virtual const Error *NextSegment( uunit pos[], uunit vel[], uint8 &time );

   /**
       Start a new trajectory.  This function is called before the first call to 
       LinkTrajectory::NextSegment.  It will result in a call to Path::Reset

       @return An error pointer if the trajectory object is not available, or NULL
               if it is ready to be sent.
    */
   virtual const Error *StartNew( void );

   /**
     Play back path data.  This method may be used to itterate through a path 
     for display purposes.

     Before starting a path playback, the path should be reset using the method
     Path::Reset.

     Each call to this function will return position and velocity information for 
     the current playback position in the path.  It will then increment the playback 
     position by the time value passed.  When the end of the path is reached, the
     method will return true.

     @param timeInc The amount of time (seconds) to increment the playback position 
                    after reading out the position & velocity values.
     @param pos An array where the position information will be returned.  This array
                    must be long enough to store DIM elements, where DIM is the path
		    dimension.
     @param vel An array where the velocity information will be returned.  This array
                    must be long enough to store DIM elements, where DIM is the path
		    dimension.
     @return true if the end of the path has been reached, false if not.
    */
   bool PlayPath( double timeInc, double pos[], double vel[] );
};

CML_NAMESPACE_END()
#endif

