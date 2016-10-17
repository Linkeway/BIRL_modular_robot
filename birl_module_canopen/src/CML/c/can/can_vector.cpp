/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/* CAN object for Vector Can Card X */

#define DYNAMIC_CANDRIVER_DLL
#include <VCanD.h>

#include "CML.h"
#include <can_vector.h>
#include <windows.h>
#include <cstdio>

CML_NAMESPACE_USE();

/* local data - these are pointers to the functions defined in the Vector .dll file */
static const char *dllName = "vcand32.dll";
static HMODULE hDLL = 0;
static int openCards = 0;
static Mutex libraryMutex;

static NCDOPENDRIVER             ncdDllOpenDriver;
static NCDCLOSEDRIVER            ncdDllCloseDriver;
static NCDGETCHANNELMASK         ncdGetChannelMask;
static NCDOPENPORT               ncdOpenPort;
static NCDACTIVATECHANNEL        ncdActivateChannel;
static NCDSETNOTIFICATION        ncdSetNotification;
static NCDGETERRORSTRING         ncdGetErrorString;
static NCDTRANSMIT               ncdTransmit;
static NCDRECEIVE1               ncdReceive1;
static NCDRECEIVE                ncdReceive;
static NCDREQUESTCHIPSTATE       ncdRequestChipState;
static NCDSETCHANNELACCEPTANCE   ncdSetChannelAcceptance;
static NCDDEACTIVATECHANNEL      ncdDeactivateChannel;
static NCDCLOSEPORT              ncdClosePort;
static NCDSETCHANNELBITRATE      ncdSetChannelBitrate;
static NCDSETCHANNELMODE         ncdSetChannelMode;
static NCDGETAPPLCONFIG          ncdGetApplConfig;
static NCDRESETCLOCK             ncdResetClock;
static NCDFLUSHRECEIVEQUEUE      ncdFlushReceiveQueue;
static NCDGETEVENTSTRING         ncdGetEventString;
static NCDSETAPPLCONFIG          ncdSetApplConfig;
static NCDSETCHANNELPARAMS       ncdSetChannelParams;
static NCDSETCHANNELPARAMSC200   ncdSetChannelParamsC200;
static NCDGETRECEIVEQUEUELEVEL   ncdGetReceiveQueueLevel;
static NCDSETTIMERRATE           ncdSetTimerRate;
static NCDGETDRIVERCONFIG        ncdGetDriverConfig;
static NCDRESETACCEPTANCE        ncdResetAcceptance;
static NCDADDACCEPTANCERANGE     ncdAddAcceptanceRange;
static NCDREMOVEACCEPTANCERANGE  ncdRemoveAcceptanceRange;
static NCDGETCHANNELINDEX        ncdGetChannelIndex;
static NCDSETCHANNELOUTPUT       ncdSetChannelOutput;
static NCDSETCHANNELTRANSCEIVER  ncdSetChannelTransceiver;
static NCDFLUSHTRANSMITQUEUE     ncdFlushTransmitQueue;
static NCDGETSTATE               ncdGetState;
static NCDGETCHANNELVERSION      ncdGetChannelVersion;
static NCDSETRECEIVEMODE         ncdSetReceiveMode;

/* external functions */
extern const Error *WaitOnWindowsObject( HANDLE hndl, int32 timeout );

/* local data structure */
typedef struct
{
   /// Event handle used to communicate events from the driver.
   HANDLE event;

   /// Mask used to identify the CAN channel to the driver
   Vaccess mask;

   /// Handle used to access the CAN port
   VportHandle handle;
} LocalData;

/* local functions */
static const Error *ConvertError( Vstatus err );
static const Error *InitLibrary( void );
static void UninitLibrary( void );

/***************************************************************************/
/**
Construct a default CAN object.
The CAN interface is closed initially, and no port name is selected.
*/
/***************************************************************************/
VectorCAN::VectorCAN( void ) : CanInterface()
{
   // Default baud to 1,000,000 bps
   SetBaud( 1000000 );

   // Default to not open
   open = 0;

   // Don't allow sharing
   shareOK = false;

   // Allocate memory for local data
   data = new LocalData;
}

/***************************************************************************/
/**
Construct a CAN object with a specified port name.
The port name should be of the form CANx or VECTORx where x is the port number.
The port numbers start at 0, so the first port would be identified by
the port name CAN0.
@param port The port name string identifying the CAN device.
*/
/***************************************************************************/
VectorCAN::VectorCAN( const char *port ) : CanInterface(port)
{
   // Default baud to 1,000,000 bps
   baud = 1000000;

   // Default to not open
   open = 0;

   // Don't allow sharing
   shareOK = false;

   // Allocate memory for local data
   data = new LocalData;
}

/***************************************************************************/
/**
Close the CAN port and unload the dll.
*/
/***************************************************************************/
VectorCAN::~VectorCAN( void )
{
   Close();

   if( data ) delete data;
}

/***************************************************************************/
/**
Open the Vector CAN port.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *VectorCAN::Open( void )
{
   LocalData *ldat = (LocalData *)data;
   int err;
   int hwType, hwIndex, hwChannel;
   int port;

   if( !data )
      return &CanError.Alloc;

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
      port = FindPortNumber( "VECTOR" );

   if( port < 0 )
   {
      mutex.Unlock();
      return &CanError::BadPortName;
   }

   const Error *iniErr = InitLibrary();
   if( iniErr )
   {
      cml.Error( "VectorCAN::InitLibrary failed with error: %s\n", iniErr->toString() );
      mutex.Unlock();
      return iniErr;
   }

   /**************************************************
    * Open the driver.
    **************************************************/
   err = ncdDllOpenDriver();

   /**************************************************
    * Get a list of available CAN hardware
    **************************************************/
   int ct;

   if( !err ) err = ncdGetDriverConfig( &ct, 0 );

   if( !err && !ct ) err = VERR_HW_NOT_PRESENT;

   if( !err )
   {
      VDriverConfig *cfg = new VDriverConfig[ ct ];
      ncdGetDriverConfig( &ct, cfg );

      int j;
      for( j=0; j<cfg->channelCount; j++ )
      {
	 // Ignore virtual channels
	 if( cfg->channel[j].hwType <= HWTYPE_VIRTUAL )
	    continue;

	 if( !port-- )
	    break;
      }

      if( j == cfg->channelCount )
	 err = VERR_HW_NOT_PRESENT;

      else
      {
	 hwType = cfg->channel[j].hwType;
	 hwIndex = cfg->channel[j].hwIndex;
	 hwChannel = cfg->channel[j].hwChannel;
      }

      delete cfg;
   }

   if( !err )
   {
      // Find the channel mask for this channel
      ldat->mask = ncdGetChannelMask( hwType, hwIndex, hwChannel );

      // If no mask was found, there is no such hardware available.
      if( !ldat->mask )
	 err = VERR_HW_NOT_PRESENT;
   }

   Vaccess permission;

   // Open the port
   if( !err ) 
      err = ncdOpenPort( &ldat->handle, "MotionLib", ldat->mask, ldat->mask, &permission, 1024 );

   if( !err && !permission && !shareOK )
      err = VERR_CHAN_IS_ONLINE;

   // Set the channel baud rate
   if( !err && permission )
      err = ncdSetChannelBitrate( ldat->handle, ldat->mask, baud );

   // Set normal output mode
   if( !err && permission )
      err = ncdSetChannelOutput( ldat->handle, ldat->mask, OUTPUT_MODE_NORMAL );

   // Enable all messages
   if( !err )
   {
      VsetAcceptance accept;
      accept.code = 0;
      accept.mask = 0;
      err = ncdSetChannelAcceptance( ldat->handle, ldat->mask, &accept );
   }

   // Tell the port to notify me immediately if any data is received
   if( !err )
   {
      ldat->event = CreateEvent( NULL, FALSE, FALSE, NULL );
      err = ncdSetNotification( ldat->handle, (unsigned long *)&ldat->event, 1 );
   }

   // Reset the clock used to time events
   if( !err )
      err = ncdResetClock( ldat->handle );

   // Disable the TX and TXRQ notifications
   if( !err )
      err = ncdSetChannelMode( ldat->handle, ldat->mask, 0, 0 );

   if( !err )
      err = ncdActivateChannel( ldat->handle, ldat->mask );

   if( !err )
      open = 1;
   else
      UninitLibrary();

   mutex.Unlock();

   return ConvertError(err);
}

/***************************************************************************/
/**
Close the CAN interface.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *VectorCAN::Close( void )
{
   LocalData *ldat = (LocalData *)data;
   int err;

   mutex.Lock();
   if( !open )
   {
      return &CanError::NotOpen;
      mutex.Unlock();
   }

   open = 0;
   err = ncdClosePort( ldat->handle );

   if( !err ) err = ncdDllCloseDriver();

   UninitLibrary();

   mutex.Unlock();

   return ConvertError(err);;
}

/***************************************************************************/
/**
Set the CAN interface baud rate.
@param b The baud rate to set.
@return A CAN error object identifying the error.
*/
/***************************************************************************/
const Error *VectorCAN::SetBaud( int32 b )
{
   if( b < 5000 || b > 1000000 )
      return &CanError.BadParam;

   mutex.Lock();
   baud = b;
   mutex.Unlock();

   return 0;
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
const Error *VectorCAN::RecvFrame( CanFrame &frame, int32 timeout )
{
   // See if there is any message in my receive queue right now
   LocalData *ldat = (LocalData *)data;
   int err, ct, i;
   Vevent *pEvent;

   if( !open ) 
      return &CanError::NotOpen;

   ncdGetReceiveQueueLevel( ldat->handle, &ct );

   if( timeout < 0 ) timeout = INFINITE;

   while( 1 )
   {
      // See if there is anything available on the receive queue.
      mutex.Lock();
      err = ncdReceive1( ldat->handle, &pEvent );

      if( err )
      {
	 mutex.Unlock();

	 const Error *eptr = WaitOnWindowsObject( ldat->event, timeout );
	 if( eptr ) return eptr;

	 continue;
      }

      // Handle the event on the receive queue
      switch( pEvent->tag )
      {
	 // New receive frame
	 case V_RECEIVE_MSG:

	    if( pEvent->tagData.msg.id & 0x80000000 )
	       frame.id = 0x20000000 | (pEvent->tagData.msg.id & 0x1FFFFFFF);
	    else
	       frame.id = pEvent->tagData.msg.id & 0x07FF;

	    if( pEvent->tagData.msg.flags & MSGFLAG_ERROR_FRAME )
	       frame.type = CAN_FRAME_ERROR;

	    else if( pEvent->tagData.msg.flags & MSGFLAG_REMOTE_FRAME )
	       frame.type = CAN_FRAME_REMOTE;

	    else
	       frame.type = CAN_FRAME_DATA;

	    frame.length = pEvent->tagData.msg.dlc;

	    if( frame.length > 8 ) frame.length = 8;

	    for( i=0; i<frame.length; i++ )
	       frame.data[i] = pEvent->tagData.msg.data[i];

	    mutex.Unlock();
	    return 0;

	 case V_CHIP_STATE:
	    // Chip state change
	    cml.Warn( "Vector: Chip state - status: 0x%02x, txErr: %d, rxErr: %d\n", 
		  pEvent->tagData.chipState.busStatus,
		  pEvent->tagData.chipState.txErrorCounter,
		  pEvent->tagData.chipState.rxErrorCounter );
	    break;

	 default:
	    // Some other unexpected event
	    cml.Debug( "Vector: CAN event %d received\n", pEvent->tag );
	    break;
      }

      mutex.Unlock();
   }
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
const Error *VectorCAN::XmitFrame( CanFrame &frame, int32 timeout )
{
   LocalData *ldat = (LocalData *)data;

   if( frame.length > 8 )
      return &CanError.BadParam;

   if( !open ) return &CanError::NotOpen;

   Vevent event;

   event.tag = V_TRANSMIT_MSG;
   event.tagData.msg.id = frame.id;

   switch( frame.type )
   {
      case CAN_FRAME_DATA:
	 event.tagData.msg.flags = 0;
	 break;

      case CAN_FRAME_REMOTE:
	 event.tagData.msg.flags = MSGFLAG_REMOTE_FRAME;
	 break;

      default:
	 return &CanError.BadParam;
   }

   event.tagData.msg.dlc = frame.length;

   for( int i=0; i<frame.length; i++ )
      event.tagData.msg.data[i] = frame.data[i];

   mutex.Lock();
   const Error *err = ConvertError( ncdTransmit( ldat->handle, ldat->mask, &event ) );
   mutex.Unlock();

   return err;
}

/***************************************************************************/
/**
Convert error codes defined by the Vector CAN library into 
the standard error codes used by the motion library.
@param err The Vector style status code
@return A CAN error object.
*/
/***************************************************************************/
static const Error *ConvertError( Vstatus err )
{
   switch( err )
   {
      case VSUCCESS:                     return 0;
      case VERR_QUEUE_IS_EMPTY:          return &CanError.Overflow;
      case VERR_QUEUE_IS_FULL:           return &CanError.Overflow;
      case VERR_TX_NOT_POSSIBLE:         return &CanError.Driver;
      case VERR_NO_LICENSE:              return &CanError.Driver;
      case VERR_WRONG_PARAMETER:         return &CanError.BadParam;
      case VERR_TWICE_REGISTER:          return &CanError.BadParam;
      case VERR_INVALID_CHAN_INDEX:      return &CanError.BadPortName;
      case VERR_INVALID_ACCESS:          return &CanError.BadParam;
      case VERR_PORT_IS_OFFLINE:         return &CanError.Driver;
      case VERR_CHAN_IS_ONLINE:          return &CanError.AlreadyOpen;
      case VERR_NOT_IMPLEMENTED:         return &CanError.BadParam;
      case VERR_INVALID_PORT:            return &CanError.BadPortName;
      case VERR_HW_NOT_READY:            return &CanError.Driver;
      case VERR_CMD_TIMEOUT:             return &CanError.Timeout;
      case VERR_HW_NOT_PRESENT:          return &CanError.BadPortName;
      case VERR_NOTIFY_ALREADY_ACTIVE:   return &CanError.BadParam;
      case VERR_NO_RESOURCES:            return &CanError.BadParam;
      case VERR_WRONG_CHIP_TYPE:         return &CanError.BadParam;
      case VERR_WRONG_COMMAND:           return &CanError.BadParam;
      case VERR_INVALID_HANDLE:          return &CanError.BadParam;
      case VERR_CANNOT_OPEN_DRIVER:      return &CanError.Driver;
      case VERROR:                       return &CanError.Unknown;
      default:                           return &CanError.Unknown;
   }
}

/***************************************************************************/
/**
Initialize the Vector .dll function pointers.  This internal method is called
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
      hDLL = LoadLibrary( dllName );
      if( !hDLL )
      {
	 cml.Error( "Unable to load library file: %s\n", dllName );
	 err = &CanError::NoDriver;
      }
      else
      {
	 ncdDllOpenDriver         = (NCDOPENDRIVER)            GetProcAddress( hDLL, "ncdOpenDriver"            );
	 ncdDllCloseDriver        = (NCDCLOSEDRIVER)           GetProcAddress( hDLL, "ncdCloseDriver"           );
	 ncdOpenPort              = (NCDOPENPORT)              GetProcAddress( hDLL, "ncdOpenPort"              );
	 ncdActivateChannel       = (NCDACTIVATECHANNEL)       GetProcAddress( hDLL, "ncdActivateChannel"       );
	 ncdSetNotification       = (NCDSETNOTIFICATION)       GetProcAddress( hDLL, "ncdSetNotification"       );
	 ncdGetErrorString        = (NCDGETERRORSTRING)        GetProcAddress( hDLL, "ncdGetErrorString"        );
	 ncdTransmit              = (NCDTRANSMIT)              GetProcAddress( hDLL, "ncdTransmit"              );
	 ncdReceive1              = (NCDRECEIVE1)              GetProcAddress( hDLL, "ncdReceive1"              );
	 ncdReceive               = (NCDRECEIVE)               GetProcAddress( hDLL, "ncdReceive"               );
	 ncdSetChannelAcceptance  = (NCDSETCHANNELACCEPTANCE)  GetProcAddress( hDLL, "ncdSetChannelAcceptance"  );
	 ncdDeactivateChannel     = (NCDDEACTIVATECHANNEL)     GetProcAddress( hDLL, "ncdDeactivateChannel"     );
	 ncdClosePort             = (NCDCLOSEPORT)             GetProcAddress( hDLL, "ncdClosePort"             );
	 ncdGetChannelMask        = (NCDGETCHANNELMASK)        GetProcAddress( hDLL, "ncdGetChannelMask"        );
	 ncdSetChannelBitrate     = (NCDSETCHANNELBITRATE)     GetProcAddress( hDLL, "ncdSetChannelBitrate"     );
	 ncdSetChannelMode        = (NCDSETCHANNELMODE)        GetProcAddress( hDLL, "ncdSetChannelMode"        );
	 ncdGetApplConfig         = (NCDGETAPPLCONFIG)         GetProcAddress( hDLL, "ncdGetApplConfig"         );
	 ncdResetClock            = (NCDRESETCLOCK)            GetProcAddress( hDLL, "ncdResetClock"            );
	 ncdFlushReceiveQueue     = (NCDFLUSHRECEIVEQUEUE)     GetProcAddress( hDLL, "ncdFlushReceiveQueue"     );
	 ncdGetEventString        = (NCDGETEVENTSTRING)        GetProcAddress( hDLL, "ncdGetEventString"        );
	 ncdSetApplConfig         = (NCDSETAPPLCONFIG)         GetProcAddress( hDLL, "ncdSetApplConfig"         );
	 ncdSetChannelParamsC200  = (NCDSETCHANNELPARAMSC200)  GetProcAddress( hDLL, "ncdSetChannelParamsC200"  );
	 ncdGetReceiveQueueLevel  = (NCDGETRECEIVEQUEUELEVEL)  GetProcAddress( hDLL, "ncdGetReceiveQueueLevel"  );
	 ncdSetTimerRate          = (NCDSETTIMERRATE)          GetProcAddress( hDLL, "ncdSetTimerRate"          );
	 ncdGetDriverConfig       = (NCDGETDRIVERCONFIG)       GetProcAddress( hDLL, "ncdGetDriverConfig"       );
	 ncdGetChannelIndex       = (NCDGETCHANNELINDEX)       GetProcAddress( hDLL, "ncdGetChannelIndex"       );
	 ncdSetChannelOutput      = (NCDSETCHANNELOUTPUT)      GetProcAddress( hDLL, "ncdSetChannelOutput"      );
	 ncdSetChannelTransceiver = (NCDSETCHANNELTRANSCEIVER) GetProcAddress( hDLL, "ncdSetChannelTransceiver" );
	 ncdSetChannelParams      = (NCDSETCHANNELPARAMS)      GetProcAddress( hDLL, "ncdSetChannelParams"      );
	 ncdRequestChipState      = (NCDREQUESTCHIPSTATE)      GetProcAddress( hDLL, "ncdRequestChipState"      );
	 ncdFlushTransmitQueue    = (NCDFLUSHTRANSMITQUEUE)    GetProcAddress( hDLL, "ncdFlushTransmitQueue"    );
	 ncdGetState              = (NCDGETSTATE)              GetProcAddress( hDLL, "ncdGetState"              );
	 ncdGetChannelVersion     = (NCDGETCHANNELVERSION)     GetProcAddress( hDLL, "ncdGetChannelVersion"     );
	 ncdSetReceiveMode        = (NCDSETRECEIVEMODE)        GetProcAddress( hDLL, "ncdSetReceiveMode"        );
	 ncdAddAcceptanceRange	 = (NCDADDACCEPTANCERANGE)    GetProcAddress( hDLL, "ncdAddAcceptanceRange"    );
	 ncdRemoveAcceptanceRange = (NCDREMOVEACCEPTANCERANGE) GetProcAddress( hDLL, "ncdRemoveAcceptanceRange" );
	 ncdResetAcceptance	    = (NCDRESETACCEPTANCE)       GetProcAddress( hDLL, "ncdResetAcceptance"       );

	 if( !ncdDllOpenDriver || !ncdDllCloseDriver || !ncdOpenPort || !ncdActivateChannel || 
	       !ncdSetNotification || !ncdGetErrorString || !ncdTransmit || !ncdReceive1 || 
	       !ncdReceive || !ncdSetChannelAcceptance || !ncdDeactivateChannel || !ncdClosePort || 
	       !ncdGetChannelMask || !ncdSetChannelBitrate || !ncdSetChannelMode || !ncdGetApplConfig || 
	       !ncdResetClock || !ncdFlushReceiveQueue || !ncdGetEventString || !ncdSetApplConfig || 
	       !ncdSetChannelParamsC200 || !ncdGetReceiveQueueLevel || !ncdSetTimerRate || 
	       !ncdGetDriverConfig || !ncdGetChannelIndex || !ncdSetChannelOutput || 
	       !ncdSetChannelTransceiver || !ncdSetChannelParams || !ncdRequestChipState || 
	       !ncdFlushTransmitQueue || !ncdGetState || !ncdGetChannelVersion || !ncdSetReceiveMode || 
	       !ncdAddAcceptanceRange || !ncdRemoveAcceptanceRange || !ncdResetAcceptance )
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

