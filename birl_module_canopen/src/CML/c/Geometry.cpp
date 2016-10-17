/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
*/

#include "CML_Settings.h"
#include "CML.h"

CML_NAMESPACE_USE();

#ifdef CML_ALLOW_FLOATING_POINT

#include <math.h>


uunit PointN::distance( PointN &p )
{
   CML_ASSERT( getDim() == p.getDim() );

   double dist = 0;
   int dim = getDim();

   for( int i=0; i<dim; i++ )
   {
      double x = getPos(i) - p.getPos(i);
      dist += x*x;
   }

   return (uunit)sqrt(dist);
}
#endif

PointN &PointN::operator+=( PointN &p )
{
   CML_ASSERT( getDim() == p.getDim() );

   int dim = getDim();
   for( int i=0; i<dim; i++ )
      setPos( i, getPos(i) + p.getPos(i) );
   return *this;
}

PointN &PointN::operator-=( PointN &p )
{
   CML_ASSERT( getDim() == p.getDim() );

   int dim = getDim();
   for( int i=0; i<dim; i++ )
      setPos( i, getPos(i) - p.getPos(i) );
   return *this;
}

