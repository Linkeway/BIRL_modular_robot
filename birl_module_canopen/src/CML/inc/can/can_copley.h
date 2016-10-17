/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the Copley Controls CAN card

*/

#ifndef _DEF_INC_CAN_COPLEY
#define _DEF_INC_CAN_COPLEY

#include "CML_Settings.h"
#include "CML_Can.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/**
This class extends the generic CanInterface class into a working
interface for the Copley can device driver.

*/
class CopleyCAN : public CanInterface
{
public:
   CopleyCAN( void );
   CopleyCAN( const char *port );
   virtual ~CopleyCAN( void );
   const Error *Open( const char *name ){
      portName = name;
      return Open();
   }
   const Error *Open( void );
   const Error *Close( void );
   const Error *SetBaud( int32 baud );

protected:
   const Error *RecvFrame( CanFrame &frame, int32 timeout );
   const Error *XmitFrame( CanFrame &frame, int32 timeout );

   /// tracks the state of the interface as open or closed.
   int open;

   /// Holds a copy of the last baud rate set
   int32 baud;

   /// This pointer is used to keep track of private data
   /// used by the driver.
   void *local;
};

CML_NAMESPACE_END()

#endif

