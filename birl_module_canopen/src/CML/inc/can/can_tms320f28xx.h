/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

CAN hardware interface for the Texas Instruments 
TMS320F28xx family of DSPs.
*/


#ifndef _DEF_INC_CAN_F28xx
#define _DEF_INC_CAN_F28xx

#include "CML_Settings.h"
#include "CML_can.h"

#include <CML_Threads.h>

// Receive queue length
#define F28xxCAN_RQLEN  20

CML_NAMESPACE_START()

// CAN mailbox used internally
struct F28xx_Mailbox
{
	int32 time;          // Timestamp used for receive messages
	uint32 id;           // CAN message ID
	uint32 ctrl;         // Mailbox control bits
	uint32 data[2];      // Message data
	struct F28xx_Mailbox *next;
	struct F28xx_Mailbox *prev;
};

/**
TMS320F28xx specific CAN interface.

This class extends the generic CanInterface class into a working
interface for the 'F28xx can device driver.

*/

class F28xxCAN : public CanInterface
{
public:
	F28xxCAN( void );
	F28xxCAN( const char *port );
	virtual ~F28xxCAN( void );

	const Error *Open( const char *name ){
		portName = name;
		return Open();
	}
	const Error *Open( void );
	const Error *Close( void );
	const Error *SetBaud( int32 baud );

	void HandleInt( void );

protected:
	const Error *RecvFrame( CanFrame &frame, int32 timeout );
	const Error *XmitFrame( CanFrame &frame, int32 timeout );

	/// Holds a copy of the last baud rate set
	int32 baud;

private:
	/// Mutex for internal use
	Mutex mutex;

	/// Semaphore used for transmit control
	Semaphore xmitSem;

	/// Next transmit message priority
	volatile int nextXmitPri;

	/// Semaphore used for receive control
	Semaphore recvSem;

	/// Bit timing configuration 
	uint32 btc;

	/// Pointer to the CAN port registers
	void *regPtr;

	/// Receive queue
	F28xx_Mailbox *head, *tail, *free;
	F28xx_Mailbox recvQ[ F28xxCAN_RQLEN ];
};

CML_NAMESPACE_END()

#endif




