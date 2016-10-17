/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file contains class definitions used to define multi-axis
trajectory paths.
*/


#ifndef _DEF_INC_GEOMETRY
#define _DEF_INC_GEOMETRY

#include "CML_Settings.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
An N axis point.  This point specifies a position in N dimensions.

This is a pure virtual base class of the more specific Point classes.
*/
/***************************************************************************/
class PointN
{
public:

   /// Virtual destructor 
   virtual ~PointN(){};

   /// Get the number of dimensions of this point
   /// @return The point dimension
   virtual int getDim( void ) const = 0;

   /// Set the number of dimensions of this point
   /// @param d The new point dimension
   virtual void setDim( int d ) = 0;

   /// Get the max dimensions that this point can handle
   /// @return The max value.
   virtual int getMax( void ) const = 0;

   virtual uunit getPos( int i ) = 0;

   virtual void setPos( int i, uunit p ) = 0;

#ifdef CML_ALLOW_FLOATING_POINT
   uunit distance( PointN &p );
#endif

   virtual uunit &operator[]( int i ) = 0;
   virtual uunit operator[]( int i ) const = 0;

   virtual PointN &operator+=( PointN &p );
   virtual PointN &operator-=( PointN &p );
};

/***************************************************************************/
/**
Template used for N dimensional objects.  This template may be used to 
generate point objects for some fixed number of dimensions.
*/
/***************************************************************************/
template<int N> class Point: public PointN
{
   uunit pos[N];
   int dim;
public:
   Point( void )
   {
      dim = N;
      for( int i=0; i<N; i++ ) 
	 pos[i] = 0;
   }

   Point( const PointN &p )
   {
      setDim( p.getDim() );
      for( int i=0; i<dim; i++ )
	 pos[i] = p[i];
   }

   Point &operator=( const PointN &p )
   {
      setDim( p.getDim() );
      for( int i=0; i<dim; i++ )
	 pos[i] = p[i];
      return *this;
   }

   int getDim( void ) const { return dim; }
   int getMax( void ) const { return N; }
   void setDim( int d ){ CML_ASSERT(d<=N); dim = d; }
   uunit getPos( int i ){ CML_ASSERT( (i>=0) && (i<N) ); return pos[i]; }
   void setPos( int i, uunit p ){ CML_ASSERT( (i>=0) && (i<N) ); pos[i]=p; }
   uunit &operator[]( int i ){ CML_ASSERT( (i>=0) && (i<N) ); return pos[i]; }
   uunit operator[]( int i ) const { CML_ASSERT( (i>=0) && (i<N) ); return pos[i]; }
};

CML_NAMESPACE_END()
#endif

