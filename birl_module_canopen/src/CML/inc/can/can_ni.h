/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the National Instruments CAN card.

*/

#ifndef _DEF_INC_CAN_NI
#define _DEF_INC_CAN_NI

#include "CML_Settings.h"
#include "CML_Can.h"
#include "CML_Threads.h"

CML_NAMESPACE_START();

/**
National Instruments specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the National Instruments CAN card.

*/
class NI_Can: public CanInterface
{
public:
	NI_Can( void );
	NI_Can( const char *portName );

	virtual ~NI_Can( void );

	const Error *Open( void );
	const Error *Close( void );
	const Error *SetBaud( int32 baud );

protected:
	const Error *RecvFrame( CanFrame &frame, int32 timeout );
	const Error *XmitFrame( CanFrame &frame, int32 timeout );

	/// tracks the state of the interface as open or closed.
	int open;

	/// Holds the handle value used to communicate with the
	/// National Instruments API.
   uint32 handle;

	/// Holds a copy of the last baud rate set
	int32 baud;

	Mutex mutex;
};

CML_NAMESPACE_END();

#endif

