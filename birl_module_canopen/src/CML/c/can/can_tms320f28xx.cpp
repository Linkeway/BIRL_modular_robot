/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/*
	CAN object for Texas Instruments TMS320F28xx DSP on board CAN controller.
	This module is expected to run with TI's DSPBIOS operating system.
*/

#include <std.h>
#include <hwi.h>
#include <c28.h>
#include "can/can_tms320f28xx.h"
#include "CML.h"

CML_NAMESPACE_USE();

typedef volatile unsigned long LREG;
typedef volatile unsigned int REG;

/* local defines */
#define CAN_ADDRESS           0x6000
#define RECV_MB_CT            24
#define PCLKCR                (REG*)0x701C
#define GPFMUX                (REG*)0x70D4
#define PIE_BASE              0x0CE0

typedef struct
{
	LREG  id;           // CAN message ID
	LREG  ctrl;         // Mailbox control bits
	LREG  data[2];      // Message data
} CAN_MB;

typedef struct
{
	LREG  ME;           // 0x6000 - Mailbox enable
	LREG  MD;           // 0x6002 - Mailbox direction
	LREG  TRS;          // 0x6004 - Transmit request set
	LREG  TRR;          // 0x6006 - Transmit request reset
	LREG  TA;           // 0x6008 - Transmission acknowledge
	LREG  AA;           // 0x600A - Abort acknowledge
	LREG  RMP;          // 0x600C - Receive message pending
	LREG  RML;          // 0x600E - Receive message lost
	LREG  RFP;          // 0x6010 - Remote frame pending
	LREG  rsvd1;        // 0x6012 - Reserved
	LREG  MC;           // 0x6014 - Master control
	LREG  BTC;          // 0x6016 - Bit-timing configuration
	LREG  ES;           // 0x6018 - Error and status
	LREG  TEC;          // 0x601A - Transmit error counter
	LREG  REC;          // 0x601C - Receive error counter
	LREG  GIF0;         // 0x601E - Global interrupt flag 0
	LREG  GIM;          // 0x6020 - Global interrupt mask
	LREG  GIF1;         // 0x6022 - Global interrupt flag 1
	LREG  MIM;          // 0x6024 - Mailbox interrupt mask
	LREG  MIL;          // 0x6026 - Mailbox interrupt level
	LREG  OPC;          // 0x6028 - Overwrite protection control
	LREG  TIOC;         // 0x602A - TX I/O control
	LREG  RIOC;         // 0x602C - RX I/O control
	LREG  LNT;          // 0x602E - Local network time (Reserved in SCC mode)
	LREG  TOC;          // 0x6030 - Time-out control (Reserved in SCC mode)
	LREG  TOS;          // 0x6032 - Time-out status (Reserved in SCC mode)
	LREG  rsvd2[6];     // 0x6034 - Reserved
	LREG  LAM[32];      // 0x6040 - Local Acceptance Masks
	LREG  MOTS[32];     // 0x6080 - Message Object Time Stamps
	LREG  MOTO[32];     // 0x60C0 - Message Object Time-Outs
	CAN_MB Mailbox[32]; // 0x6100 - CAN Mailboxes
} CAN_REGS;

typedef struct
{
	REG CTRL;
	REG ACK;
	struct
	{
		REG ENA;
		REG FLG;
	} VECT[12];
} PIE_REGS;

// Macros used to enable/disable writing to some registers
#define EnableRegAccess()    asm( " eallow" );
#define DisableRegAccess()   asm( " edis" );

// Local functions 
static uint32 CanRegRead( LREG &addr );
static void CanRegWrite( LREG &ptr, uint32 data );

/* local data */
static F28xxCAN *openPort = 0;

/***************************************************************************/
/**
Construct a default CAN object.
The CAN interface is closed initially, and no port address is specified.
*/
/***************************************************************************/
F28xxCAN::F28xxCAN( void ) : CanInterface()
{
	// Default baud to 1,000,000 bps
	SetBaud( 1000000 );

	// Default to not open
	regPtr = 0;
}

/***************************************************************************/
/**
Construct a CAN object with a specified port name.
Note that the port name is currently not used by this CAN 
interface class.
@param port Name of the CAN port to access.
*/
/***************************************************************************/
F28xxCAN::F28xxCAN( const char *port ) : CanInterface(port)
{
	// Default baud to 1,000,000 bps
	SetBaud( 1000000 );

	// Default to not open
	regPtr = 0;
}

/***************************************************************************/
/**
Destructor.  This closes the CAN port.
*/
/***************************************************************************/
F28xxCAN::~F28xxCAN( void ) 
{
	Close();
}

/***************************************************************************/
/**
Open the CAN port.  Before Open is called, the desired baud rate and
port address should be set.

If the baud rate is not explicitly set, the 1,000,000 bits/set will be
used.  If the port address isn't set, then the default address will be
used.

@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/
const Error *F28xxCAN::Open( void )
{
	int i;
	mutex.Lock();

Thread::sleep(1);
	// See if this port has already been open
	if( regPtr || openPort )
	{
		mutex.Unlock();
		return &CanError::AlreadyOpen;
	}

	// Initialize semaphores and receive queue
	free = head = tail = 0;
	for( i=0; i<F28xxCAN_RQLEN; i++ )
	{
		recvQ[i].next = free;
		free = &recvQ[i];
	}
	
	while( !recvSem.Get(0) );

	// Initialize the transmit semaphore to the number
	// of free transmit mailboxes
	while( !xmitSem.Get(0) );
	for( i=0; i<32-RECV_MB_CT; i++ )
		xmitSem.Put();
	nextXmitPri = 31;

	/**************************************************
	* Enable the peripheral clock used by the CAN module
	**************************************************/
	EnableRegAccess();
	*PCLKCR |= 0x4000;

	/**************************************************
	* Assign the CAN pins to the CAN port.  These are
	* GPIO by default on reset.
	**************************************************/
	*GPFMUX |= 0x00C0;

	/**************************************************
	* Find the address of the CAN port registers
	**************************************************/
	CAN_REGS *regs = (CAN_REGS *)CAN_ADDRESS;

	// Reset the CAN hardware and enter configuration mode
	CanRegWrite( regs->MC, 0x00001020 );
	for( i=0; !(CanRegRead(regs->ES) & 0x10) && (i<1000); i++ )
		Thread::sleep(1);

	if( !(CanRegRead(regs->ES) & 0x10) )
	{
		DisableRegAccess();
		mutex.Unlock();
		return &CanError::Unknown;
	}

	// Select HECC mode
	CanRegWrite( regs->MC, 0x00003000 );

	uint32 allRecv = (1L<<RECV_MB_CT) - 1;

	CanRegWrite( regs->MD,   allRecv );      // Use the top 12 for transmit
	CanRegWrite( regs->BTC,  btc );          // Set the bit timing config 
	CanRegWrite( regs->MIM,  0xFFFFFFFF );   // Enable all mailbox interrupts
	CanRegWrite( regs->GIM,  0x00003F01 );   // Use interrupt line 0 for all ints
	CanRegWrite( regs->OPC,  allRecv );      // Don't overwrite messages
	CanRegWrite( regs->TIOC, 8 );            // Configure transmit pin
	CanRegWrite( regs->RIOC, 8 );            // Configure receive pin

	// Setup the acceptance masks to allow all messages
	for( i=0; i<32; i++ )
		CanRegWrite( regs->LAM[i], 0x9FFFFFFF );

	// Clear the control field of each mailbox
	// and set the AME bit (Acceptance mask enable)
	for( i=0; i<32; i++ )
	{
		CanRegWrite( regs->Mailbox[i].ctrl, 0 );
		CanRegWrite( regs->Mailbox[i].id, 0x40000000 );
	}
	
	// Enable all receive mailboxes
	CanRegWrite( regs->ME, allRecv );

	// Exit configuration mode
	CanRegWrite( regs->MC, 0x00002080 );

	// Enable peripherial interrupt vector 9.5 which
	// is the vector used by the CAN controller.
	PIE_REGS *pie = (PIE_REGS *)PIE_BASE;
	pie->VECT[8].ENA |= 0x10;

	// Enable the processor interrupt 
	C28_enableIER( 0x0100 );
	DisableRegAccess();

	for( i=0; (CanRegRead(regs->ES) & 0x10) && (i<1000); i++ )
		Thread::sleep(1);

	if( CanRegRead(regs->ES) & 0x10 )
	{
		mutex.Unlock();
		return &CanError::Unknown;
	}

	regPtr = regs;
	openPort = this;
	mutex.Unlock();

	return 0;
}

/***************************************************************************/
/**
Close the CAN interface.
@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/
const Error *F28xxCAN::Close( void )
{
	mutex.Lock();
	CAN_REGS *regs = (CAN_REGS *)regPtr;
	regPtr = 0;
	mutex.Unlock();

	if( !regs )
		return 0;

	openPort = 0;

	// Enter configuration mode which disables the port
	EnableRegAccess();
	CanRegWrite( regs->MC, 0x00001000 );
	DisableRegAccess();

	return 0;
}


/***************************************************************************/
/**
Set the CAN interface baud rate.
@param b The baud rate to set.
@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/
const Error *F28xxCAN::SetBaud( int32 b )
{
	//-----------------------------------------------------------------------------------
	// Configure the bit timing register based on the desired baud rate.
	//
	// The CAN bit timing is made up of a number of different fields:
	//
	//   BRP   - Baud Rate Prescaler.  This is a divider used to reduce the
	//           input clock (150 MHz) down to a more reasonable rate.
	//           The period of the divided down clock is known as a Time Quanta (TQ)
	//   TSEG1 - Length of the first programmable portion of the CAN bit time in TQ
	//   TSEG2 - Length of the second programmable portion of the bit time in TQ
	//
	//  The actual bit time will be an integer number of time quanta long.
	//  The actual length of a bit is TSEG1 + TSEG2 + 1.  The extra 1 is not 
	//  programmable and is called the synch segment.
	//
	//  I'll be using bit times based on 15 TQ / bit.  TSEG1 will be 11 TQ long and
	//  TSEG2 will be 3 TQ long.  This puts the sample point (which is between TSEG1
	//  and TSEG2) at 80% of the bit time, which is generally considered a good location.
	//-----------------------------------------------------------------------------------
	long brp;
	int tseg1, tseg2;

	switch( b )
	{
		case 1000000: brp =  10; tseg1 = 11; tseg2 = 3; break;
		case  800000: brp =  12; tseg1 = 11; tseg2 = 4; break;
		case  500000: brp =  20; tseg1 = 11; tseg2 = 3; break;
		case  250000: brp =  40; tseg1 = 11; tseg2 = 3; break;
		case  125000: brp =  80; tseg1 = 11; tseg2 = 3; break;
		case   50000: brp = 200; tseg1 = 11; tseg2 = 3; break;
		case   20000: brp = 500; tseg1 = 11; tseg2 = 3; break;
		default: 
			return &CanError::BadParam;
	}

	//-----------------------------------------------------------
	// A couple other parameters also effect the bit rate setup.
	//  SAM - if set, take multiple bus samples.  BRP must be > 4.
	//  SJW - Synch jump width.  Max number of TQ to adjust the bit
	//        timing to align the edges of the bit.  Can be 1 to 4
	//        TQ, but can't be greater then TSEG2
	//-----------------------------------------------------------
	int sam = (brp>4) ? 1 : 0;
	int sjw = (tseg2>4 ) ? 4 : tseg2;

	// Build up the actual value to be loaded into the CAN register
	btc = 0x00000400;
	btc |= (0x07 & (tseg2-1));
	btc |= (0x0F & (tseg1-1)) << 3;
	btc |= (0x01 & sam) << 7;
	btc |= (0x03 & (sjw-1)) << 8;
	btc |= (0xFF & (brp-1)) << 16;

	baud = b;
	return 0;
}

// Utility function used to split a 32-bit value into 4 bytes.
static void Split( uint32 l, byte *bptr )
{
	bptr[0] = 0x00FF & (l >> 24);
	bptr[1] = 0x00FF & (l >> 16);
	bptr[2] = 0x00FF & (l >> 8);
	bptr[3] = 0x00FF & (l);
}

// Utility function used to pack 4 bytes into a 32-bit value
static uint32 Pack( byte *bptr )
{
	uint32 l;
	l  = (uint32)(bptr[0]&0x00FF) << 24;
	l |= (uint32)(bptr[1]&0x00FF) << 16;
	l |= (uint32)(bptr[2]&0x00FF) << 8;
	l |= (uint32)(bptr[3]&0x00FF);
	return l;
}

/***************************************************************************/
/**
Receive the next CAN frame.  
@param frame A reference to the frame object that will be filled by the read.
@param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
       return immediately if no data is available.  A timeout of < 0 will 
		 wait forever.
@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/
const Error *F28xxCAN::RecvFrame( CanFrame &frame, int32 timeout )
{
	// Make sure the port is open
	if( !regPtr )
		return &CanError::NotOpen;

	// Wait for the new message
	const Error *err = recvSem.Get(timeout);
	if( err ) return err;

	mutex.Lock();

	// Copy the last message in the queue
	HWI_disable();
	F28xx_Mailbox *mbPtr = head;
	head = head->next;

	if( !head )
		tail = 0;
	else
		head->prev = 0;

	HWI_enable();

	frame.type = (mbPtr->ctrl & 0x10) ?  CAN_FRAME_REMOTE : CAN_FRAME_DATA;

	if( mbPtr->id & 0x80000000 )
		frame.id = 0x20000000 | (mbPtr->id & 0x1FFFFFFF);
	else
		frame.id = 0x000007FF & (mbPtr->id >> 18);

	frame.length = mbPtr->ctrl & 0x0F;
	
	Split( mbPtr->data[0], &(frame.data[0]) );
	Split( mbPtr->data[1], &(frame.data[4]) );

	// Free the queue location
	HWI_disable();
	mbPtr->next = free;
	free = mbPtr;
	HWI_enable();

	mutex.Unlock();
	return 0;
}

/***************************************************************************/
/**
Write a CAN frame to the CAN network.
@param frame A reference to the frame to write.
@param timeout The time to wait for the frame to be successfully sent.
       If the timeout is 0, the frame is written to the output queue and
		 the function returns without waiting for it to be sent.
		 If the timeout is <0 then the function will delay forever.
@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/
const Error *F28xxCAN::XmitFrame( CanFrame &frame, int32 timeout )
{
	// Make sure the port is open
	CAN_REGS *regs = (CAN_REGS *)regPtr;
	if( !regs ) 
		return &CanError::NotOpen;

	// Wait for a free transmit mailbox
	const Error *err = xmitSem.Get(timeout);
	if( err ) 
		return err;

	mutex.Lock();
	
	// The transmit mailboxes on this chip implement a priority
	// mechanism that I don't use.  To ensure that I send my 
	// messages in the correct order I need to send each message
	// out at a priority level lower then the last.  When I hit 
	// zero priority, I need to wait for the interrupt handler to 
	// reset my priority to the max.
	while( nextXmitPri < 0 )
	{
		mutex.Unlock();
		if( !timeout )
			return &ThreadError::Timeout;	
		Thread::sleep(1);
		if( timeout > 0 ) timeout--;
		mutex.Lock();
	}

	// Find a pointer to a free mailbox
	uint32 mask = 0x80000000;
	uint32 me = CanRegRead( regs->ME );
	int i;
	for( i=31; (me & mask) && (i>=RECV_MB_CT); i--, mask>>=1 );

	// Make sure we found a free mailbox.  This should always be true,
	// if it isn't there must be a bug in this driver.
	if( i < RECV_MB_CT )
	{
		mutex.Unlock();
		return &CanError::Driver;
	}

	// Copy the message into the mailbox
	CAN_MB *mb = &regs->Mailbox[i];

	if( frame.id & 0x20000000 )
		CanRegWrite( mb->id, (0x1FFFFFFF & frame.id) | 0x80000000 );
	else
		CanRegWrite( mb->id, (0x000007FF & frame.id) << 18 );

	uint32 ctrl = (frame.length <= 8) ? frame.length : 8;

	if( frame.type == CAN_FRAME_REMOTE )
		ctrl |= 0x10;

	CanRegWrite( mb->data[0], Pack( &frame.data[0] ) );
	CanRegWrite( mb->data[1], Pack( &frame.data[4] ) );

	HWI_disable();
	ctrl |= (nextXmitPri << 8);
	nextXmitPri--;

	CanRegWrite( mb->ctrl, ctrl );

	// Start the transmit
	me = CanRegRead( regs->ME ) | mask;
	CanRegWrite( regs->ME, me );
	CanRegWrite( regs->TRS, mask );
	HWI_enable();
	mutex.Unlock();
	
	return 0;
}

/***************************************************************************/
/**
CAN Hardware Interrupt handler.

It's assumed that the DSPBIOS configuration has been setup so that this 
function's address is listed in the vector table for the correct interrupt.
*/
/***************************************************************************/
void F28xxCAN_IntHandler( void )
{
	// Acknowledge this interrupt
	PIE_REGS *pie = (PIE_REGS *)PIE_BASE;
	pie->ACK = 0x100;

	if( openPort ) openPort->HandleInt();
}

void F28xxCAN::HandleInt( void )
{
	int i;
	CAN_REGS *regs = (CAN_REGS *)regPtr;

	uint32 ints;

	while( (ints=CanRegRead(regs->GIF0)) != 0 )
	{
		if( !(ints & 0x8000) )
			CanRegWrite( regs->GIF0, ints );
		
		// Check for a new mailbox interrupt 
		// (transmit or receive)
		else
		{
			i = ints & 0x1F;
			uint32 mask = 1;
			mask <<= i;

			// Handle receive message
			if( i < RECV_MB_CT )
			{
				// If there's space in my receive queue, 
				// copy the received message there.
				// If not, I'm going to loose it.
				F28xx_Mailbox *mb = free;
				if( !mb )
				{
					CanRegWrite( regs->RMP, mask );
					continue;
				}

				free = free->next;
				mb->time    = CanRegRead( regs->MOTS[i] );
				mb->id      = CanRegRead( regs->Mailbox[i].id );
				mb->ctrl    = CanRegRead( regs->Mailbox[i].ctrl );
				mb->data[0] = CanRegRead( regs->Mailbox[i].data[0] );
				mb->data[1] = CanRegRead( regs->Mailbox[i].data[1] );
				recvSem.Put();

				if( !tail )
				{
					head = tail = mb;
					mb->prev = mb->next = 0;
				}
				else if( mb->time - tail->time > 0 )
				{
					mb->next = 0;
					mb->prev = tail;
					tail->next = mb;
					tail = mb;
				}
				else if( mb->time - head->time < 0 )
				{
					mb->next = head;
					mb->prev = 0;
					head->prev = mb;
					head = mb;
				}
				else
				{
					F28xx_Mailbox *old = tail;

					while( mb->time - old->time < 0 )
						old = old->prev;

					mb->prev = old;
					mb->next = old->next;

					old->next = mb;
					mb->next->prev = mb;
				}

				// Release the mailbox
				CanRegWrite( regs->RMP, mask );
			}

			// Handle transmit done
			else
			{
				CanRegWrite( regs->TA, mask );
				xmitSem.Put();

				// Disable any mailboxes which are now finished
				uint32 enable = CanRegRead(regs->ME) & ~mask;
				CanRegWrite( regs->ME, enable );

				// If there are no transmits pending, then reset the next
				// transmit priority to the max value.
				if( !CanRegRead(regs->TRS) )
					nextXmitPri = 31;
			}
		}
	}

	return;
}

/***************************************************************************/
/**
 * Read from a CAN register.  We can't just read from the registers like 
 * memory because of Errata in the chip.
*/
/***************************************************************************/
static uint32 CanRegRead( LREG &ptr )
{
	// Read the register twice with ints disabled
	// Return the first read if non-zero.  If the
	// first read was zero, return the second.
	asm( " push st1" );
	asm( " setc intm" );
	uint32 x = ptr;
	if( !x ) x = ptr;
	asm( " pop st1" );

	return x;
}

/***************************************************************************/
/**
 * Write to a CAN register.  We can't just write to the registers like 
 * memory because of Errata in the chip.
*/
/***************************************************************************/
static void CanRegWrite( LREG &ptr, uint32 data )
{
	asm( " push st1" );
	asm( " setc intm" );
	ptr = data;
	ptr = data;
	asm( " pop st1" );
}

