/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the TrjTcurve class.  This class is used to 
calculate T-curve trajectory profiles for use by the Amp or Linkage objects.
*/

#ifndef _DEF_INC_TRJTCURVE
#define _DEF_INC_TRJTCURVE

#include "CML_Settings.h"
#include "CML_Trajectory.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This class represents error conditions that can occur in the TrjTcurve class.
*/
/***************************************************************************/
class TcurveError: public Error
{
public:
   static const TcurveError BadParam;            ///< Illegal input parameter
   static const TcurveError NoCalc;              ///< Trjaectory has not been calculated
   static const TcurveError InUse;               ///< Trajectory is currently in use
   static const TcurveError NotInUse;            ///< Trajectory has not been started

protected:
   /// Standard protected constructor
   TcurveError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**

*/
/***************************************************************************/
class TrjTcurve: public Trajectory 
{
   bool inUse;
   bool init;
   
   double P, V, A;
   double  ta, tv,td;

   uunit start;
   uunit end;

   double lastSegVel;
   
   uunit segNum;

   int seg;
   
   double p,v,a;
   double remain;

   int AdvanceSegment( double &tf );
   TrjTcurve( const TrjTcurve& );
   TrjTcurve& operator=( const TrjTcurve& );

public:
   TrjTcurve();

   void  SetStartPos( uunit s );
   uunit GetStartPos( void );
   uunit GetLastSegmentPos(void);
   double GetLastSegmentVel(void);
   void SetSegmentNumber(uunit num);
   bool IsEndSegment(void);
   const Error *Calculate( uunit start, uunit end, uunit vel, uunit acc);
   const Error *Calculate( uunit dist, uunit vel, uunit acc);
   const Error *StartNew( void );
   void Finish( void );
   const Error *NextSegment( uunit &pos, uunit &vel, uint8 &time );
};

CML_NAMESPACE_END()

#endif

