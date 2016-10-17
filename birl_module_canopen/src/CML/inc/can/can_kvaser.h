/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the Kvaser CAN driver

*/



#ifndef _DEF_INC_CAN_KVASER
#define _DEF_INC_CAN_KVASER

#include "CML_Settings.h"
#include "CML_can.h"

#include <CML_Threads.h>

CML_NAMESPACE_START();

/**
Kvaser specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the Kvaser can device driver.

*/

class KvaserCAN : public CanInterface
{
public:
	KvaserCAN( void );
	KvaserCAN( const char *port );
	virtual ~KvaserCAN( void );

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

	/// Holds a value the Kvaser driver uses to identify bit rate
	int kvBaud;

	/// File handle used to access the CAN channel
	int Handle;

	const Error *ConvertError( int err );

	Mutex mutex;

private:
	/// Mutex used by reading threads to ensure clean exit
	Mutex readMutex;

	/// Counter used by reading threads
	int readCount;

	void IncReadCount( int ct );
	void WaitReadCount( void );
};

CML_NAMESPACE_END();

#endif



