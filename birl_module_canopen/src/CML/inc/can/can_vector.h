/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the Vector Can Card-X

*/

#ifndef _DEF_INC_CAN_VECTOR
#define _DEF_INC_CAN_VECTOR

#define _cplusplus

#include "CML_Settings.h"
#include "CML_Can.h"
#include "CML_Threads.h"

CML_NAMESPACE_START();

/**
Vector specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the Vector CAN cards.

*/
class VectorCAN: public CanInterface
{
public:
	VectorCAN( void );
	VectorCAN( const char *port );
	virtual ~VectorCAN( void );
	const Error *Open( void );
	const Error *Close( void );
	const Error *SetBaud( int32 baud );

	/// If this method is called before the CanInterface is opened,
	/// then the Open will succeed even if another process already
	/// has the port open.  This was added to allow the CAN card to 
	/// be used by CML even when it's open by an external logging
	/// program.  Note that the baud rate can not be set when this
	/// is set.
	void  AllowSharedAccess( void )
	{
		shareOK = true;
	}

protected:
	const Error *RecvFrame( CanFrame &frame, int32 timeout );
	const Error *XmitFrame( CanFrame &frame, int32 timeout );

	/// Allow port sharing if true.
	bool shareOK;

	/// tracks the state of the interface as open or closed.
	int open;

	/// Holds a copy of the last baud rate set
	long baud;

	/// Pointer used to access some local data
	void *data;

	Mutex mutex;
};

CML_NAMESPACE_END();

#endif


