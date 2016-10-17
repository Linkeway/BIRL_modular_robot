/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/*
   CAN object for KVASER CAN driver
   (see http://www.kvaser.com for more information on the KVASER driver for the PCIcanS card)
   */

#include <cstdio>
#include <errno.h>
#include <canlib.h>
#include "can_kvaser.h"
#include "CML.h"

/* external functions */
extern void CheckWindowsThreadStop( void );

/* Types used to define functions contained in the Kvaser .dll files */
typedef canStatus (CANLIBAPI *canBusOnType)( int );
typedef canStatus (CANLIBAPI *canSetBusParamsType)( int, long, uint, uint, uint, uint, uint );
typedef canStatus (CANLIBAPI *canIoCtlType)( int, uint, void*, uint );
typedef canStatus (CANLIBAPI *canOpenChannelType)( int, int );
typedef canStatus (CANLIBAPI *canInitLibraryType)( void );
typedef canStatus (CANLIBAPI *canCloseType)( int );
typedef canStatus (CANLIBAPI *canBusOffType)( int );
typedef canStatus (CANLIBAPI *canReadWaitType)( int, long*, void*, uint*, uint*, ulong*, ulong );
typedef canStatus (CANLIBAPI *canReadType)( int, long*, void*, uint*, uint*, ulong* );
typedef canStatus (CANLIBAPI *canWriteType)( int, long, void*, uint, uint );

CML_NAMESPACE_USE();

/* local functions */
static const Error *InitLibrary( void );
static void UninitLibrary( void );

/* local data */
static const char *dllName = "canlib32.dll";
static HMODULE hDLL = 0;
static int openCards = 0;
static Mutex libraryMutex;

static canBusOnType LPcanBusOn;
static canSetBusParamsType LPcanSetBusParams;
static canIoCtlType LPcanIoCtl;
static canOpenChannelType LPcanOpenChannel;
static canInitLibraryType LPcanInitLibrary;
static canCloseType LPcanClose;
static canBusOffType LPcanBusOff;
static canReadWaitType LPcanReadWait;
static canReadType LPcanRead;
static canWriteType LPcanWrite;

/***************************************************************************/
/**
  Construct a default CAN object.
  The CAN interface is closed initially, and no port name is selected.
  */
/***************************************************************************/
KvaserCAN::KvaserCAN( void ) : CanInterface()
{
   // Default baud to 1,000,000 bps
   SetBaud( 1000000 );

   // Default to not open
   open = 0;
   readCount = 0;
}

/***************************************************************************/
/**
  Construct a CAN object with a specified port name.
  The port name should be of the form CANx or KVASERx where x is the port number.
  The port numbers start at 0, so the first port would be identified by
  the port name CAN0.
  @param port The port name string identifying the CAN device.
  */
/***************************************************************************/
KvaserCAN::KvaserCAN( const char *port ) : CanInterface(port)
{
   // Default baud to 1,000,000 bps
   SetBaud( 1000000 );

   // Default to not open
   open = 0;
   readCount = 0;
}

/***************************************************************************/
/**
  Destructor.  This closes the CAN port and frees the .dll 
  */
/***************************************************************************/
KvaserCAN::~KvaserCAN( void ) 
{
   Close();
}

/***************************************************************************/
/**
  Open the Kvaser CAN port.  Before Open is called, the desired baud rate must
  have been specified by calling SetBaud, and the port name must have been
  set.  If the baud was not specified, it will default to 1,000,000 BPS.  If 
  the port name is not set, it will default to CAN0.

  @return A pointer to an error object on failure, NULL on success.
  */
/***************************************************************************/
const Error *KvaserCAN::Open( void )
{
   int port;

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
   if( port < 0 ) port = FindPortNumber( "KVASER" );
   if( port < 0 )
   {
      mutex.Unlock();
      return &CanError::BadPortName;
   }

   const Error *err = InitLibrary();
   if( err )
   {
      mutex.Unlock();
      cml.Error( "KvaserCAN::InitLibrary failed with error: %s\n", err->toString() );
      return err;
   }

   LPcanInitLibrary();

   Handle = LPcanOpenChannel( port, canWANT_EXCLUSIVE);

   int status = LPcanIoCtl(Handle, canIOCTL_FLUSH_RX_BUFFER, NULL, NULL);

   // Set the baud rate 
   if( status == canOK ) status = LPcanSetBusParams(Handle, kvBaud, 0, 0, 0, 0, 0 );

   // Set the acceptance mask (not yet supported) ?
   if( status == canOK ) status = LPcanBusOn(Handle);
   open = 1;

   // Set the acceptance mask (not yet supported) ?
   mutex.Unlock();
   return ConvertError( status );
}

/***************************************************************************/
/**
  Close the CAN interface.
  @return A pointer to an error object on failure, NULL on success.
  */
/***************************************************************************/
const Error *KvaserCAN::Close( void )
{
   mutex.Lock();
   int wasOpen = open;
   open = 0;
   mutex.Unlock();

   if( !wasOpen )
      return 0;

   // Wait for reading threads to time out.
   WaitReadCount();

   mutex.Lock();
   canStatus status = LPcanBusOff(Handle);
   status = LPcanClose(Handle);
   mutex.Unlock();

   // Free the DLL if no longer in use.
   UninitLibrary();

   return ConvertError(status);
}


/***************************************************************************/
/**
  Set the CAN interface baud rate.
  @param b The baud rate to set.
  @return A pointer to an error object on failure, NULL on success.
  */
/***************************************************************************/
const Error *KvaserCAN::SetBaud( int32 b )
{
   switch( b )
   {
      case 1000000: kvBaud = BAUD_1M;   break;
      case  500000: kvBaud = BAUD_500K; break;
      case  250000: kvBaud = BAUD_250K; break;
      case  125000: kvBaud = BAUD_125K; break;
      case  100000: kvBaud = BAUD_100K; break;
      case   62000: kvBaud = BAUD_62K;  break;
      case   50000: kvBaud = BAUD_50K;  break;
      default: 
		    return &CanError::BadParam;
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
  @return A pointer to an error object on failure, NULL on success.
  */
/***************************************************************************/
const Error *KvaserCAN::RecvFrame( CanFrame &frame, int32 timeout )
{
   canStatus       status;
   uint    dlc, flags;
   ulong   time;

   if( !open ) 
      return &CanError::NotOpen;

   // Do an immediate read if no timeout is requested
   if( timeout == 0 )
   {
      IncReadCount( 1 );
      try {
	 status = LPcanRead( Handle, (long*)&frame.id, frame.data, &dlc, &flags, &time );
      }
      catch( ... ) {
	 IncReadCount( -1 );
	 throw;
      }
      IncReadCount( -1 );
   }

   // For reading with a timeout, do a bunch of reads with a max timeout of 100 ms.
   // This allows me to check to make sure the thread wasn't stopped (Thread::stop)
   // while I was waiting.  This is kind of ugly, but it seems to be the only way 
   // to allow the thread to be stopped using this driver.
   else
   {
      while( timeout )
      {
	 if( !open ) 
	    return &CanError::NotOpen;

	 // Check for Thread::stop.
	 CheckWindowsThreadStop();

	 // Wait for up to 100 ms
	 int32 to = timeout;

	 if( to < 0 ) 
	    to = 100;

	 else if( to > 100 )
	 {
	    to = 100;
	    timeout -= 100;
	 }

	 else
	    timeout = 0;

	 IncReadCount( 1 );
	 try {
	    status = LPcanReadWait( Handle, (long*)&frame.id, frame.data, &dlc, &flags, &time, to );
	 }
	 catch( ... ) {
	    IncReadCount( -1 );
	    throw;
	 }

	 IncReadCount( -1 );
	 if( status != canERR_NOMSG )
	    break;
      }
   }

   if( status < 0 )
      return ConvertError( status );

   // indicate an extended frame by turning on bit 29 of the frame id
   if( flags & canMSG_EXT )
   {
      frame.id &= 0x1FFFFFFF;
      frame.id |= 0x20000000;
   }
   else
      frame.id &= 0x000007FF;

   // Log error flags
   if( flags & canMSGERR_MASK )
      cml.Warn( "Kvaser driver error: 0x%04x\n", flags );

   // Set the frame type
   if( flags & canMSG_ERROR_FRAME )
      frame.type = CAN_FRAME_ERROR;

   else if( flags & canMSG_RTR )
      frame.type = CAN_FRAME_REMOTE;

   else
      frame.type = CAN_FRAME_DATA;

   frame.length = dlc;

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
const Error *KvaserCAN::XmitFrame( CanFrame &frame, int32 timeout )
{
   canStatus status;
   long id;
   uint dlc, flags;

   // don't allow frame lengths longer than 8
   if( frame.length > 8 )
      return &CanError::BadParam;

   id = frame.id & 0x1FFFFFFF;
   dlc = frame.length;

   switch( frame.type )
   {
      case CAN_FRAME_DATA:
	 flags = 0;
	 break;

      case CAN_FRAME_REMOTE:
	 flags = canMSG_RTR;
	 break;

      default:
	 return &CanError::BadParam;
   }

   // set the extended frame status
   if( frame.id & 0x20000000 )
      flags |= canMSG_EXT;
   else
      flags |= canMSG_STD;

   mutex.Lock();

   if( !open ) 
   {
      mutex.Unlock();
      return &CanError::NotOpen;
   }

   status = LPcanWrite(Handle, id, frame.data, dlc, flags);
   mutex.Unlock();

   return ConvertError( status );
}

/***************************************************************************/
/**
  Convert error codes defined by the Vector CAN library into 
  the standard error codes used by the motion library.
  @param err The Vector style status code
  @return A pointer to an error object on failure, NULL on success.
  */
/***************************************************************************/
const Error *KvaserCAN::ConvertError( int err )
{
   switch( err )
   {
      case canOK:                      return 0;
      case canERR_PARAM:               return &CanError::BadPortName; // Error in parameter (correct?)
      case canERR_NOMSG:               return &CanError::Timeout;     // No messages available (correct?)
      case canERR_NOTFOUND:            return &CanError::Driver;      // Specified hw not found (more detail?)
      case canERR_NOMEM:               return &CanError::Driver;      // Out of memory (more detail?)
      case canERR_NOCHANNELS:          return &CanError::BadPortName; // No channels avaliable
      case canERR_RESERVED_3:          return &CanError::Driver;
      case canERR_TIMEOUT:             return &CanError::Timeout;     // Timeout ocurred
      case canERR_NOTINITIALIZED:      return &CanError::Driver;      // Lib not initialized (more detail?)
      case canERR_NOHANDLES:           return &CanError::Driver;      // Can't get handle (more detail?)
      case canERR_INVHANDLE:           return &CanError::Driver;      // Handle is invalid (more detail?)
      case canERR_INIFILE:             return &CanError::Driver;      // Error in the ini-file (16-bit only) (more detail?)
      case canERR_DRIVER:              return &CanError::Driver;      // CAN driver type not supported
      case canERR_TXBUFOFL:            return &CanError::Driver;      // Transmit buffer overflow
      case canERR_RESERVED_1:          return &CanError::Driver;      // (more detail?)
      case canERR_HARDWARE:            return &CanError::Driver;      // Some hardware error has occurred (more detail?)
      case canERR_DYNALOAD:            return &CanError::Driver;      // Can't find requested DLL (more detail?)
      case canERR_DYNALIB:             return &CanError::Driver;      // DLL seems to be wrong version (more detail?)
      case canERR_DYNAINIT:            return &CanError::Driver;      // Error when initializing DLL
      case canERR_RESERVED_4:          return &CanError::Driver;      // (more detail)
      case canERR_RESERVED_5:          return &CanError::Driver;      // (more detail)
      case canERR_RESERVED_6:          return &CanError::Driver;      // (more detail)
      case canERR_RESERVED_2:          return &CanError::Driver;      // (more detail)
      case canERR_DRIVERLOAD:          return &CanError::Driver;      // Can't find/load driver (more detail)
      case canERR_DRIVERFAILED:        return &CanError::Driver;      // DeviceIOControl failed; use Win32 GetLastError() (more detail)
      case canERR_NOCONFIGMGR:         return &CanError::Driver;      // Can't find req'd config s/w (e.g. CS/SS) (more detail)
      case canERR_NOCARD:              return &CanError::Driver;      // The card was removed or not inserted (more detail)
      case canERR_RESERVED_7:          return &CanError::Driver;
      case canERR_REGISTRY:            return &CanError::Driver;      // Error in the Registry (more detail)
      case canERR_LICENSE:             return &CanError::Driver;      // The license is not valid. (more detail)
      case canERR_INTERNAL:            return &CanError::Driver;      // Internal error in the driver. (more detail)
      default:
				       return &CanError::Unknown;
   }
}

/***************************************************************************/
/**
  Increment the local read counter by the specified amount.  The read counter
  is used to keep track of how many threads are reading from the driver.  This
  allows me to exit the driver cleanly.
  @param ct The amount to add to the read counter (normally 1 or -1)
  */
/***************************************************************************/
void KvaserCAN::IncReadCount( int ct )
{
   readMutex.Lock();
   readCount += ct;
   readMutex.Unlock();
}

/***************************************************************************/
/**
  Wait for the read counter to reach zero.  This is used when the driver is
  closed to ensure a clean exit.
  */
/***************************************************************************/
void KvaserCAN::WaitReadCount( void )
{
   int ct;
   while( 1 )
   {
      readMutex.Lock();
      ct = readCount;
      readMutex.Unlock();

      if( ct <= 0 )
	 return;

      Thread::sleep( 50 );
   }
}

/***************************************************************************/
/**
  Initialize the local pointers to the Kvaser .dll file.
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
      // Load the Kvaser supplied .dll file 
      hDLL = LoadLibrary( dllName );

      if( !hDLL )
      {
	 cml.Error( "Unable to load library file: %s\n", dllName );
	 err = &CanError::NoDriver;
      }
      else
      {
	 LPcanBusOn        = (canBusOnType)       GetProcAddress( hDLL, "canBusOn"             );
	 LPcanSetBusParams = (canSetBusParamsType)GetProcAddress( hDLL, "canSetBusParams"      );
	 LPcanIoCtl        = (canIoCtlType)       GetProcAddress( hDLL, "canIoCtl"             );
	 LPcanOpenChannel  = (canOpenChannelType) GetProcAddress( hDLL, "canOpenChannel"       );
	 LPcanInitLibrary  = (canInitLibraryType) GetProcAddress( hDLL, "canInitializeLibrary" );
	 LPcanClose        = (canCloseType)       GetProcAddress( hDLL, "canClose"             );
	 LPcanBusOff       = (canBusOffType)      GetProcAddress( hDLL, "canBusOff"            );
	 LPcanReadWait     = (canReadWaitType)    GetProcAddress( hDLL, "canReadWait"          );
	 LPcanRead         = (canReadType)        GetProcAddress( hDLL, "canRead"              );
	 LPcanWrite        = (canWriteType)       GetProcAddress( hDLL, "canWrite"             );

	 if( !LPcanBusOn || !LPcanSetBusParams || !LPcanIoCtl || 
	       !LPcanOpenChannel || !LPcanInitLibrary || !LPcanClose || 
	       !LPcanBusOff || !LPcanReadWait || !LPcanRead || !LPcanWrite )
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

