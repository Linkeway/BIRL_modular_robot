/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the Ixxat CAN driver

*/

#ifndef _DEF_INC_CAN_IXXAT
#define _DEF_INC_CAN_IXXAT

#include "CML_Settings.h"
#include "CML_can.h"
#include "CML_Threads.h"

CML_NAMESPACE_START();

/// This gives the size of the CAN message receive queue
/// used for Ixxat cards
#define IXXAT_RX_QUEUE_SZ      50

/**
Ixxat specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the Ixxat can device driver.
*/

class IxxatCAN : public CanInterface
{
public:
	IxxatCAN( void );
	IxxatCAN( const char *port );
	virtual ~IxxatCAN( void );
	const Error *Open( void );
	const Error *Close( void );
	const Error *SetBaud( int32 baud );

	void rxInt( int16 ct, void *frame );

protected:
	const Error *RecvFrame( CanFrame &frame, int32 timeout );
	const Error *XmitFrame( CanFrame &frame, int32 timeout );

	CanFrame rx[ IXXAT_RX_QUEUE_SZ ];
	volatile int rxHead, rxTail;
	Semaphore rxSem;

	/// tracks the state of the interface as open or closed.
	int open;

	/// Which CAN channel to use (on multi-channel boards).
	/// For the moment this is always set to zero.
	uint8 channel;

	/// Holds a copy of the last baud rate set
	int32 baud;

	uint8 bt0, bt1;

	/// File handle used to access the CAN channel
	int handle;

	uint16 txQueue;
	uint16 rxQueue;

	const Error *ConvertError( int err );

	Mutex mutex;
};

CML_NAMESPACE_END();

#endif

