/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/*
	CAN object for IXXAT CAN driver
	(see http://www.ixxat.com for more information on the IXXAT VCI driver for the VCI series cards)
*/

#include <cstdio>
#include <cstring>
#include <windows.h>

#include "CML.h"
#include "can_ixxat.h"
#include "XatXXReg.h"
#include "VCI2.h"

CML_NAMESPACE_USE();

/* local functions */
static void VCI_CALLBACKATTR rxInt0( UINT16 q, UINT16 ct, VCI_CAN_OBJ *p );
static void VCI_CALLBACKATTR rxInt1( UINT16 q, UINT16 ct, VCI_CAN_OBJ *p );
static void VCI_CALLBACKATTR rxInt2( UINT16 q, UINT16 ct, VCI_CAN_OBJ *p );
static void VCI_CALLBACKATTR rxInt3( UINT16 q, UINT16 ct, VCI_CAN_OBJ *p );

/* Types used to define functions contained in the Ixxat .dll files */
typedef HRESULT (XATREG_CALLATTR *XAT_SelectHardwareType)( HWND , XAT_BoardCFG * );
typedef HRESULT (XATREG_CALLATTR *XAT_GetConfigType)( DWORD , XAT_BoardCFG * );
typedef HRESULT (XATREG_CALLATTR *XAT_GetDefaultHwEntryType)( DWORD * );
typedef INT32 (VCI_CALLATTR *VCI_StartCanType)( UINT16 , UINT8 );
typedef INT32 (VCI_CALLATTR *VCI_AssignRxQueObjType)( UINT16, UINT16, UINT8, UINT32, UINT32 );
typedef INT32 (VCI_CALLATTR *VCI_ConfigQueueType)( UINT16, UINT8, UINT8, UINT16, UINT16, UINT16, UINT16, UINT16* );
typedef INT32 (VCI_CALLATTR *VCI_SetAccMaskType)( UINT16, UINT8, UINT32, UINT32 );
typedef INT32 (VCI_CALLATTR *VCI_InitCanType)( UINT16, UINT8, UINT8, UINT8, UINT8 );
typedef INT32 (VCI_CALLATTR *VCI2_PrepareBoardType)( VCI_BOARD_TYPE, UINT16, char*, UINT8, VCI_t_PutS, VCI_t_UsrRxIntHdlr, VCI_t_UsrExcHdlr );
typedef INT32 (VCI_CALLATTR *VCI_CancelBoardType)( UINT16 );
typedef INT32 (VCI_CALLATTR *VCI_TransmitObjType)( UINT16, UINT16, UINT32, UINT8, UINT8* );
typedef INT32 (VCI_CALLATTR *VCI_RequestObjType)( UINT16, UINT16, UINT32, UINT8 );

/* local functions */
static const Error *InitLibrary( void );
static void UninitLibrary( void );

/* local data */
static VCI_t_UsrRxIntHdlr rxIntPtr[] = { rxInt0, rxInt1, rxInt2, rxInt3 };
#define MAX_BOARDS       sizeof(rxIntPtr)/sizeof(rxIntPtr[0])

static IxxatCAN *board[ MAX_BOARDS ] = {0};
static Mutex boardMutex;
static Mutex libraryMutex;
static int openCards = 0;

static HMODULE hReg;
static HINSTANCE hVCI;
static XAT_SelectHardwareType    lpXAT_SelectHardware;
static XAT_GetConfigType         lpXAT_GetConfig;
static XAT_GetDefaultHwEntryType lpXAT_GetDefaultHwEntry;
static VCI_StartCanType          lpVCI_StartCan;
static VCI_AssignRxQueObjType    lpVCI_AssignRxQueObj;
static VCI_ConfigQueueType       lpVCI_ConfigQueue;
static VCI_SetAccMaskType        lpVCI_SetAccMask;
static VCI_InitCanType           lpVCI_InitCan;
static VCI2_PrepareBoardType     lpVCI2_PrepareBoard;
static VCI_CancelBoardType       lpVCI_CancelBoard;
static VCI_TransmitObjType       lpVCI_TransmitObj;
static VCI_RequestObjType        lpVCI_RequestObj;

/***************************************************************************/
/**
Construct a new Ixxat CAN interface object.
This simply sets the default baud rate and 
marks the card as not open.
*/
/***************************************************************************/
IxxatCAN::IxxatCAN( void ) : CanInterface()
{
	// Default baud to 1,000,000 bps
	IxxatCAN::SetBaud( 1000000 );

	// Default to not open
	open = 0;
}

/***************************************************************************/
/**
Construct a new Ixxat CAN interface object for the specified port.

The port name should be of the form CANx or IXXATx where x is the port number.
The port numbers start at 0, so the first port would be identified by
the port name CAN0.

@param port The port name string identifying the CAN device.
*/
/***************************************************************************/
IxxatCAN::IxxatCAN( const char *port ) : CanInterface(port)
{
	// Default baud to 1,000,000 bps
	IxxatCAN::SetBaud( 1000000 );

	// Default to not open
	open = 0;
}

/***************************************************************************/
/**
Destructor for Ixxat card.  Closes the interface and unloads the library.
*/
/***************************************************************************/
IxxatCAN::~IxxatCAN(void)
{
	Close();
}

/***************************************************************************/
/**
Open the Ixxat CAN card.

The card should have been identified by setting it's name either
in the constructor, or by using the method CanInterface::SetName.

If no port name was set, then the default Ixxat card will be used.

If the port name is set to "select", then a dialog box will be shown
allowing the card to be selected from any installed Ixxat cards.

Otherwise, the port name should be of the form "CANx" where x is the
Ixxat hardware key number (i.e. CAN1 for hardware key 1).

@return A pointer to an error object on failure, or NULL on success.
*/
/***************************************************************************/
const Error *IxxatCAN::Open( void )
{
	int port;
	int ret = 0;

	mutex.Lock();

	if( open )
	{
		mutex.Unlock();
		return &CanError::AlreadyOpen;
	}

	/**************************************************
	* Find the port number to open.
	**************************************************/
	port = FindPortNumber( "CAN" );
	if( port < 0 ) 
		port = FindPortNumber( "IXXAT" );

	if (port < 0)
		port = FindPortNumber("IXXATV30");


	if( port < 0 )
	{
		mutex.Unlock();
		return &CanError::BadPortName;
	}

	const Error *err = InitLibrary();
	if( err )
	{
		cml.Error( "IxxatCAN::InitLibrary failed with error %s\n", err->toString() );
		mutex.Unlock();
		return err;
	}

	// Reset local variables
	rxHead = rxTail = 0;
	while( rxSem.Get(0) == 0 );

	/**************************************************
	* Find the board configuration info based on the
	* port name specified
	**************************************************/
	XAT_BoardCFG cfg;
	HRESULT res;
	DWORD key = port;
	int i = -1;

	if( port == 0 )
	{
		lpXAT_GetDefaultHwEntry( &key );
		res = lpXAT_GetConfig( key, &cfg );
	}

	else
		res = lpXAT_GetConfig( key, &cfg );

	if( res )
	{
		err = &CanError::BadPortName;
		goto done;
	}

	/**************************************************
	* Find an empty slot in the board array.
	**************************************************/
	boardMutex.Lock();
	for( i=0; i<MAX_BOARDS; i++ )
	{
		if( board[i] == NULL )
		{
			board[i] = this;
			break;
		}
	}
	boardMutex.Unlock();

	if( i == MAX_BOARDS )
	{
		i = -1;
		err = &CanError::Driver;
		goto done;
	}

	// For now, we always use channel 0.
	channel = 0;

	handle = lpVCI2_PrepareBoard( cfg.board_type, cfg.board_no, NULL, 0, 0, rxIntPtr[i], 0 );

	if( handle < 0 )
	{
		err = &CanError::Driver;
		goto done;
	}

	// initialize CAN-Controller
	ret = lpVCI_InitCan( handle, channel, bt0, bt1, VCI_11B );

	//  definition of Acceptance-Mask (define to receive all IDs)
	if( ret == VCI_OK ) 
		ret = lpVCI_SetAccMask( handle, channel, 0, 0 );

	//  definition of Transmit Queue
	if( ret == VCI_OK ) 
		ret = lpVCI_ConfigQueue( handle, channel, VCI_TX_QUE, 200, 0, 0, 0,  &txQueue );

	//  definition of Receive Queue (interrupt mode)
	if( ret == VCI_OK ) 
		ret = lpVCI_ConfigQueue( handle, channel, VCI_RX_QUE, 20, 1, 0, 100, &rxQueue );

	//  assign all ID's to the Receive Queue
	if( ret == VCI_OK ) 
		ret = lpVCI_AssignRxQueObj( handle, rxQueue, VCI_ACCEPT, 0, 0 );

	//  And now start the CAN
	if( ret == VCI_OK ) 
		ret = lpVCI_StartCan( handle, channel );

	if( ret != VCI_OK ) 
		err = ConvertError( ret );

done:

	if( err )
	{
		if( i >= 0 )
		{
			boardMutex.Lock();
			board[i] = NULL;
			boardMutex.Unlock();
		}
		UninitLibrary();
	}
	else
		open = 1;

	mutex.Unlock();

	return err;
}

/***************************************************************************/
/**
Close the CAN interface.
@return A pointer to an error object on failure, or NULL on success.
*/
/***************************************************************************/
const Error *IxxatCAN::Close( void )
{
	mutex.Lock();
	if( open )
	{
		lpVCI_CancelBoard( handle );
		boardMutex.Lock();
		for( int i=0; i<MAX_BOARDS; i++ )
		{
			if( board[i] == this )
				board[i] = NULL;
		}
		boardMutex.Unlock();

		open = 0;
		UninitLibrary();
	}
	mutex.Unlock();
	return 0;
}

/***************************************************************************/
/**
Set the CAN interface baud rate.
@param b The baud rate to set.
@return A pointer to an error object on failure, or NULL on success.
*/
/***************************************************************************/
const Error *IxxatCAN::SetBaud( int32 b )
{
	switch( b )
	{
		case   10000: bt0=0x31; bt1=0x1C; break;
		case   20000: bt0=0x18; bt1=0x1C; break;
		case   50000: bt0=0x09; bt1=0x1C; break;
		case  100000: bt0=0x04; bt1=0x1C; break;
		case  125000: bt0=0x03; bt1=0x1C; break;
		case  250000: bt0=0x01; bt1=0x1C; break;
		case  500000: bt0=0x00; bt1=0x1C; break;
		case  800000: bt0=0x00; bt1=0x16; break;
		case 1000000: bt0=0x00; bt1=0x14; break;
		default: return &CanError::BadParam;
	}

	baud = b;
	return 0;
}

/***************************************************************************/
/**
Receive the next CAN frame.  
@param frame A reference to the frame object that will be filled by the read.
@param timeout The timeout (ms) to wait for the frame.  A timeout of 0 will
       return immediately if no data is available.  A timeout of < 0 will 
		 wait forever.
@return A pointer to an error object on failure, or NULL on success.
*/
/***************************************************************************/
const Error *IxxatCAN::RecvFrame( CanFrame &frame, int32 timeout )
{
	if( !open )
		return &CanError::NotOpen;

	const Error *err = rxSem.Get( timeout );
	if( err ) return err;

	mutex.Lock();
	frame = rx[rxTail];

	int newTail = rxTail+1;
	if( newTail >= IXXAT_RX_QUEUE_SZ )
		newTail = 0;

	rxTail = newTail;
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
@return A pointer to an error object on failure, or NULL on success.
*/
/***************************************************************************/
const Error *IxxatCAN::XmitFrame( CanFrame &frame, int32 timeout )
{
   // don't allow frame lengths longer than 8
	if( frame.length > 8 )
		return &CanError::BadParam;

	mutex.Lock();

	if( !open )
	{
		mutex.Unlock();
		return &CanError::NotOpen;
	}

	int ret;

	switch( frame.type )
	{
		case CAN_FRAME_DATA:
			ret = lpVCI_TransmitObj( handle, txQueue, frame.id, frame.length, frame.data );
			break;
		case CAN_FRAME_REMOTE:
			ret = lpVCI_RequestObj( handle, txQueue, frame.id, frame.length );
			break;
		default:
			mutex.Unlock();
			return &CanError::BadParam;
	}

	mutex.Unlock();

	return ConvertError( ret );
}


/***************************************************************************/
/**
Convert error codes defined by the Vector CAN library into 
the standard error codes used by the motion library.
@param err The Vector style status code
@return A pointer to an error object, or NULL if no error is indicated
*/
/***************************************************************************/
const Error *IxxatCAN::ConvertError( int err )
{
	switch( err )
	{
		case VCI_OK:          return 0;
		case VCI_ERR:         return &CanError::Driver;
		case VCI_HWSW_ERR:    return &CanError::Driver;
		case VCI_SUPP_ERR:    return &CanError::Driver;
		case VCI_PARA_ERR:    return &CanError::Driver;
		case VCI_RES_ERR:     return &CanError::Driver;
		case VCI_QUE_ERR:     return &CanError::Driver;
		case VCI_TX_ERR:      return &CanError::Driver;
		default:              return &CanError::Unknown;
	}
}

/***************************************************************************/
/**
Receive interrupt handler.  This is an internal function that should not 
be called except by the driver.  It's used to add one or more CAN frames
to the receive buffer when they are received.

@param ct The number of frames to add
@param ptr Points to an array of ct VCI_CAN_OBJ structures.
*/
/***************************************************************************/
void IxxatCAN::rxInt( int16 ct, void *ptr )
{
	VCI_CAN_OBJ *frame = (VCI_CAN_OBJ*)ptr;

	for( int i=0; i<ct; i++ )
	{
		int newHead = rxHead+1;
		if( newHead >= IXXAT_RX_QUEUE_SZ )
			newHead = 0;
	
		if( newHead == rxTail )
			return;
	
		rx[ rxHead ].id       = frame[i].id;
		rx[ rxHead ].length   = frame[i].len;

		if( frame[i].rtr )
			rx[ rxHead ].type = CAN_FRAME_REMOTE;
		else
			rx[ rxHead ].type = CAN_FRAME_DATA;

		for( int j=0; j<8; j++ )
			rx[ rxHead ].data[j] = frame[i].a_data[j];

		rxHead = newHead;
		rxSem.Put();
	}
}

/***************************************************************************/
/**
Local interrupt handlers.  These just call the member function for the
CAN object.
*/
/***************************************************************************/
#define RX_INT(x)  static void VCI_CALLBACKATTR rxInt##x( UINT16 q, UINT16 ct, VCI_CAN_OBJ *p ) \
                   { if( board[x] ) board[x]->rxInt( ct, p ); }

RX_INT(0);
RX_INT(1);
RX_INT(2);
RX_INT(3);

/***************************************************************************/
/**
Initialize the Ixxat .dll function pointers.  This internal method is called
at startup and used to initialize some local pointers to functions in the 
Ixxat supplied .dll files.
@return A pointer to an error object or NULL on success.
*/
/***************************************************************************/
static const Error *InitLibrary( void )
{
   const Error *err = 0;
   libraryMutex.Lock();

   if( !openCards )
   {
      // Load the Ixxat .dll files
      hReg = LoadLibrary("xat11reg.dll");
      if( !hReg )
      {
	 cml.Error( "Unable to load library file: xat11reg.dll\n" );
	 err = &CanError::NoDriver;
	 goto done;
      }

      hVCI = LoadLibrary("vci11un6.dll");
      if( !hVCI )
      {
	 FreeLibrary( hReg );
	 cml.Error( "Unable to load library file: vci11un6.dll\n" );
	 err = &CanError::NoDriver;
	 goto done;
      }

      lpXAT_SelectHardware    = (XAT_SelectHardwareType)   GetProcAddress( hReg, "XAT_SelectHardware"    );
      lpXAT_GetConfig         = (XAT_GetConfigType)        GetProcAddress( hReg, "XAT_GetConfig"         );
      lpXAT_GetDefaultHwEntry = (XAT_GetDefaultHwEntryType)GetProcAddress( hReg, "XAT_GetDefaultHwEntry" );
      lpVCI_StartCan          = (VCI_StartCanType)         GetProcAddress( hVCI, "VCI_StartCan"          );
      lpVCI_AssignRxQueObj    = (VCI_AssignRxQueObjType)   GetProcAddress( hVCI, "VCI_AssignRxQueObj"    );
      lpVCI_ConfigQueue       = (VCI_ConfigQueueType)      GetProcAddress( hVCI, "VCI_ConfigQueue"       );
      lpVCI_SetAccMask        = (VCI_SetAccMaskType)       GetProcAddress( hVCI, "VCI_SetAccMask"        );
      lpVCI_InitCan           = (VCI_InitCanType)          GetProcAddress( hVCI, "VCI_InitCan"           );
      lpVCI2_PrepareBoard     = (VCI2_PrepareBoardType)    GetProcAddress( hVCI, "VCI2_PrepareBoard"     );
      lpVCI_CancelBoard       = (VCI_CancelBoardType)      GetProcAddress( hVCI, "VCI_CancelBoard"       );
      lpVCI_TransmitObj       = (VCI_TransmitObjType)      GetProcAddress( hVCI, "VCI_TransmitObj"       );
      lpVCI_RequestObj        = (VCI_RequestObjType)       GetProcAddress( hVCI, "VCI_RequestObj"        );


      if( !lpXAT_SelectHardware || !lpXAT_GetConfig || !lpXAT_GetDefaultHwEntry || 
	    !lpVCI_StartCan || !lpVCI_AssignRxQueObj || !lpVCI_ConfigQueue || 
	    !lpVCI_SetAccMask || !lpVCI_InitCan || !lpVCI2_PrepareBoard || 
	    !lpVCI_CancelBoard || !lpVCI_TransmitObj || !lpVCI_RequestObj )
      {
	 err = &CanError::NoDriver;
	 FreeLibrary( hVCI );
	 FreeLibrary( hReg );
      }
   }

   if( !err )
      openCards++;

done:
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
	{
		FreeLibrary( hVCI );
		FreeLibrary( hReg );
	}
	libraryMutex.Unlock();
}

