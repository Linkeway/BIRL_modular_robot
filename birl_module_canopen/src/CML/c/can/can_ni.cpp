/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/* CAN object for national instruments PCI-CAN card */

#include <stdio.h>
#include <windows.h>
#include <can_ni.h>
#include <cml.h>

#define __NC_NOINC_func
#include <nican.h>

CML_NAMESPACE_USE();

/* local functions */
static const Error *ConvertError( NCTYPE_STATUS err );
static const Error *InitLibrary( void );
static void UninitLibrary( void );

/* Types used to define functions contained in the NI .dll files */
typedef NCTYPE_STATUS (_NCFUNC_ *ncConfig_Type)( NCTYPE_STRING, NCTYPE_UINT32, NCTYPE_ATTRID_P, NCTYPE_UINT32_P );
typedef NCTYPE_STATUS (_NCFUNC_ *ncOpen_Type)( NCTYPE_STRING, NCTYPE_OBJH_P );
typedef NCTYPE_STATUS (_NCFUNC_ *ncClose_Type)( NCTYPE_OBJH );
typedef NCTYPE_STATUS (_NCFUNC_ *ncWait_Type)( NCTYPE_OBJH, NCTYPE_STATE, NCTYPE_DURATION, NCTYPE_STATE_P);
typedef NCTYPE_STATUS (_NCFUNC_ *ncRead_Type)( NCTYPE_OBJH, NCTYPE_UINT32, NCTYPE_ANY_P );
typedef NCTYPE_STATUS (_NCFUNC_ *ncWrite_Type)( NCTYPE_OBJH, NCTYPE_UINT32, NCTYPE_ANY_P );

/* local pointers used to access .dll functions */
static ncOpen_Type ncOpenObject;
static ncClose_Type ncCloseObject;
static ncRead_Type ncRead;
static ncWrite_Type ncWrite;
static ncConfig_Type ncConfig;
static ncWait_Type ncWaitForState;

/* local data */
static const char *dllName = "nican.dll";
static HMODULE hDLL = 0;
static int openCards = 0;
static Mutex libraryMutex;

/***************************************************************************/
/**
Construct a new National Instruments CAN Interface object.
*/
/***************************************************************************/
NI_Can::NI_Can( void ) : CanInterface()
{
	// Default to not open
	open = 0;

	// Default baud to 1,000,000 bps
	SetBaud( 1000000 );
}

/***************************************************************************/
/**
Construct a CAN object with a specified port name.
The port name should be of the form CANx or NICANx where x is the port number.
The port numbers start at 0, so the first port would be identified by
the port name CAN0.
@param port The port name string identifying the CAN device.
*/
/***************************************************************************/
NI_Can::NI_Can( const char *port ) : CanInterface(port)
{
	// Default to not open
	open = 0;

	// Default baud to 1,000,000 bps
	SetBaud( 1000000 );
}

/***************************************************************************/
/**
Destructor.  This closes the CAN port and frees the .dll 
*/
/***************************************************************************/
NI_Can::~NI_Can( void ) 
{
	Close();
}

/***************************************************************************/
/**
Open the National Instruments CAN port.  Before Open is called, the desired 
baud rate must have been specified by calling SetBaud, and the port name must 
have been set.  If the baud was not specified, it will default to 1,000,000 BPS.
If the port name is not set, it will default to CAN0.

@return A pointer to an error object on failure, NULL on success.
*/
/***************************************************************************/

const Error *NI_Can::Open( void )
{
	mutex.Lock();

	if( open )
	{
		mutex.Unlock();
		return &CanError::AlreadyOpen;
	}

	/**************************************************
	* Find the port number to open.
	**************************************************/
	int port = FindPortNumber( "CAN" );
	if( port < 0 ) port = FindPortNumber( "NI" );
	if( port < 0 )
	{
		mutex.Unlock();
		return &CanError::BadPortName;
	}

	const Error *err = InitLibrary();
	if( err )
	{
		mutex.Unlock();
		cml.Error( "NICAN::InitLibrary failed with error: %s\n", err->toString() );
		return err;
	}

	char localName[20];
	sprintf( localName, "CAN%d", port );

	NCTYPE_ATTRID ids[10];
	NCTYPE_UINT32 values[10];

	ids[0] = NC_ATTR_BAUD_RATE;      values[0] = baud;
	ids[1] = NC_ATTR_READ_Q_LEN;     values[1] = 20;
	ids[2] = NC_ATTR_WRITE_Q_LEN;    values[2] = 20;
	ids[3] = NC_ATTR_START_ON_OPEN;  values[3] = NC_TRUE;
	ids[4] = NC_ATTR_RTSI_MODE;      values[4] = NC_RTSI_NONE;
	ids[5] = NC_ATTR_CAN_MASK_STD;   values[5] = NC_CAN_MASK_STD_DONTCARE;
	ids[6] = NC_ATTR_CAN_MASK_XTD;   values[6] = NC_CAN_MASK_XTD_DONTCARE;
	ids[7] = NC_ATTR_CAN_COMP_STD;   values[7] = 0;
	ids[8] = NC_ATTR_CAN_COMP_XTD;   values[8] = 0;
	ids[9] = NC_ATTR_RX_Q_LEN;       values[9] = 50;

	/**************************************************
	* Configure the CAN port
	**************************************************/
	err = ConvertError( ncConfig( localName, 10, ids, values ) );

	/**************************************************
	* Open the port.
	**************************************************/
	if( !err )
      err = ConvertError( ncOpenObject( localName, (NCTYPE_OBJH *)&handle ) );

	if( !err )
		open = 1;
	else
		UninitLibrary();

	mutex.Unlock();

	return err;
}

/***************************************************************************/
/**
Close the CAN interface.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *NI_Can::Close( void )
{
	const Error *err = 0;
	mutex.Lock();
	if( open )
	{
		open = 0;
		err = ConvertError( ncCloseObject( handle ) );
		UninitLibrary();
	}
	mutex.Unlock();

	return err;
}

/***************************************************************************/
/**
Set the CAN interface baud rate.
@param b The baud rate to set.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *NI_Can::SetBaud( int32 b )
{
	mutex.Lock();
	const Error *err = 0;

	// Make sure the passed baud is supported by the card
	switch( b )
	{
		case   10000: baud = NC_BAUD_10K;   break;
		case  100000: baud = NC_BAUD_100K;  break;
		case  125000: baud = NC_BAUD_125K;  break;
		case  250000: baud = NC_BAUD_250K;  break;
		case  500000: baud = NC_BAUD_500K;  break;
		case 1000000: baud = NC_BAUD_1000K; break;
		default: 
			err = &CanError.BadBaud;
			break;
	}

	mutex.Unlock();

	// If the interface is already open, close it first
	// and then reopen it.  Opening it sets the baud rate.
	if( !err && open )
	{
		err = Close();

		if( !err )
			err = Open();
	}

	return err;
}

/***************************************************************************/
/**
Receive the next CAN frame.  
@param frame A reference to the frame object that will be filled by the read.
@param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
       return immediately if no data is available.  A timeout of < 0 will 
		 wait forever.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *NI_Can::RecvFrame( CanFrame &frame, int32 timeout )
{
	NCTYPE_STATE state;

	if( timeout == 0 ) 
		timeout = NC_DURATION_NONE;
	else if( timeout < 0 )
		timeout = NC_DURATION_INFINITE;

	const Error *err = ConvertError( ncWaitForState( handle, NC_ST_READ_AVAIL, 
                                                    timeout, &state ));
	if( err ) return err;

	mutex.Lock();
	NCTYPE_CAN_FRAME_TIMED niFrame;
	err = ConvertError( ncRead( handle, sizeof(niFrame), &niFrame ) );
	mutex.Unlock();
	if( err ) return err;

	frame.id     = niFrame.ArbitrationId;
	frame.length = niFrame.DataLength;
	frame.type   = (niFrame.IsRemote) ? CAN_FRAME_REMOTE : CAN_FRAME_DATA;

	if( frame.length > 8 ) frame.length = 8;

	for( int i=0; i<frame.length; i++ )
		frame.data[i] = niFrame.Data[i];

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
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *NI_Can::XmitFrame( CanFrame &frame, int32 timeout )
{
	NCTYPE_CAN_FRAME niFrame;

	if( frame.length > 8 )
		return &CanError.BadParam;

//	printf( "XMIT: 0x%08lx %d, ", frame.id, frame.length );
//	for( int j=0; j<frame.length; j++ )
//		printf( "0x%02x ", frame.data[j] );
//	printf( "\n" );

	niFrame.ArbitrationId = frame.id;
	niFrame.DataLength    = frame.length;
	niFrame.IsRemote      = (frame.type == CAN_FRAME_REMOTE);

	for( int i=0; i<frame.length; i++ )
      niFrame.Data[i] = frame.data[i];

	// Note, the National Instruments driver has a bug which causes it to fail if
	// I try to ncWaitForState on the transmit while another thread is using
	// ncWaitForState to wait on the receiver.  This very hokey code is the best
	// compramise that I can come up with until this is fixed!
	const Error *err;
	while( 1 )
	{
		mutex.Lock();
		err = ConvertError( ncWrite( handle, sizeof(NCTYPE_CAN_FRAME), &niFrame ) );
		mutex.Unlock();

		if( err != &CanError.Overflow )
			break;

		if( !timeout )
			break;

		// The queue is full, sleep for a few milliseconds and 
		// try again.  I count down the timeout but the result
		// will be that I'll end up waiting longer then the total
		// timeout due to other system delays.
		int32 delay = (timeout > 0 && timeout < 5) ? timeout : 5;
		Thread::sleep( delay );

		if( timeout > 0 ) timeout -= delay;
	}

		
//	else if( timeout < 0 )
//		timeout = NC_DURATION_INFINITE;
//
//	NCTYPE_STATE state;
//	err = ConvertError( ncWaitForState( handle, NC_ST_WRITE_SUCCESS, timeout, &state ));

	return err;
}

/***************************************************************************/
/**
Convert error codes defined by the National Instruments CAN library into 
the standard error codes used by the motion library.
@param err The National Instruments style status code
@return A CAN error object.
*/
/***************************************************************************/
static const Error *ConvertError( NCTYPE_STATUS err )
{
//	if( err )
//		printf( "Converting NI-CAN error 0x%08x\n", err );
	switch( err )
	{
      case CanSuccess:               return 0;
		case CanErrFunctionTimeout:    return &CanError.Timeout;
		case CanErrDriver:             return &CanError.Driver;
		case CanErrBadIntfName:        return &CanError.BadPortName;
		case CanErrBadParam:           return &CanError.BadParam;
		case CanErrBadHandle:          return &CanError.BadParam;
		case CanErrBadAttributeValue:  return &CanError.BadParam;
		case CanErrAlreadyOpen:        return &CanError.AlreadyOpen;
		case CanErrOverflowWrite:      return &CanError.Overflow;
		default:                       return &CanError.Unknown;
	}
}

/***************************************************************************/
/**
Initialize the local pointers to the NI .dll file.
@return A pointer to an error object or NULL on success
*/
/***************************************************************************/
static const Error *InitLibrary( void )
{
	const Error *err = 0;
	libraryMutex.Lock();

	// Init the library for the first card only
	if( !openCards )
	{
		// Load the vendor supplied .dll file 
		hDLL = LoadLibrary( dllName );

		if( !hDLL )
		{
			cml.Error( "Unable to load library file: %s\n", dllName );
			err = &CanError::NoDriver;
		}
		else
		{
			ncOpenObject   = (ncOpen_Type)  GetProcAddress( hDLL, "ncOpenObject"   ); 
			ncCloseObject  = (ncClose_Type) GetProcAddress( hDLL, "ncCloseObject"  ); 
			ncRead         = (ncRead_Type)  GetProcAddress( hDLL, "ncRead"         ); 
			ncWrite        = (ncWrite_Type) GetProcAddress( hDLL, "ncWrite"        ); 
			ncConfig       = (ncConfig_Type)GetProcAddress( hDLL, "ncConfig"       ); 
			ncWaitForState = (ncWait_Type)  GetProcAddress( hDLL, "ncWaitForState" ); 

			if( !ncOpenObject || !ncCloseObject || !ncRead || !ncWrite || 
				 !ncConfig || !ncWaitForState )
			{
				err = &CanError::NoDriver;
				FreeLibrary( hDLL );
			}
		}
	}

	if( !err )
		openCards++;

	libraryMutex.Unlock();
	return err;
}

/***************************************************************************/
/**
Free the library pointers if they are no longer accessed.
@return A pointer to an error object or NULL on success
*/
/***************************************************************************/
static void UninitLibrary( void )
{
	libraryMutex.Lock();
	if( --openCards == 0 )
		FreeLibrary( hDLL );
	libraryMutex.Unlock();
}

