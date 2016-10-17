/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include <windows.h>
#include <process.h>
#include "CML.h"

CML_NAMESPACE_USE();

/* local data */
DWORD tlsIndex = TLS_OUT_OF_INDEXES;
Mutex globalThreadMutex;

/* local classes */
struct WinThreadData
{
   // Pointer to the thread object
   Thread *thread;

   // Mutex used to synchronize access to the thread specific data
   Mutex mtx;

   // True if the thread is running.
   bool running;

   // This event is signaled when the thread is destroyed
   HANDLE killEvent;

   // The thread signals this event when it exits.
   HANDLE exitEvent;

   // The thread's handle
   HANDLE threadHandle;

   WinThreadData( Thread *t )
   {
      thread = t;
      running = false;

      // Create events used to manage the thread
      killEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
      exitEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
   }

   ~WinThreadData(){
      if( killEvent ) CloseHandle( killEvent );
      if( exitEvent ) CloseHandle( exitEvent );
   }
};

/**************************************************
* This class is used to terminate the running thread.
* It's thrown as an exception, and caught in the 
* thread starter stub.  This allows my stack to 
* unwind properly in a thread that is stopped.
**************************************************/
class ThreadExitException
{
};

/* local functions */
static void __cdecl ThreadStarter( void *arg );
static WinThreadData *GetThreadData( void );
static void KillThread( WinThreadData *tData );

/* global functions */
const Error *WaitOnWindowsObject( HANDLE hndl, int32 timeout );

/***************************************************************************/
/**
Default constructor for a new thread object.  For the pthreads version of
this class the default constructor doesn't do anything.
*/
/***************************************************************************/
Thread::Thread( void )
{
   priority = 5;
   WinThreadData *tData = new WinThreadData( this );

   // Make sure the events were created successfully
   if( !tData->killEvent || !tData->exitEvent )
   {
      delete tData;
      data = 0;
   }
   else
      data = tData;
}

/***************************************************************************/
/**
Destructor for a thread.  Causes the thread to be cancelled and waits for
it to finish.
*/
/***************************************************************************/
Thread::~Thread( void )
{
   // Stop the thread 
   const Error *err = stop(500);

   // If this fails I just return without deleting the thread data.
   // This really shouldn't happen!
   if( err )
   {
      cml.Error( "Error stopping thread in destructor: %s\n", err->toString() );
      return;
   }

   // Lock the mutex to make sure the thread really stopped
   WinThreadData *tData = (WinThreadData*)data;
   tData->mtx.Lock();

   delete data;
}

/***************************************************************************/
/**
Set the threads priority.  The priority must be specified in the range 0 
(lowest priority) to 9 (highest priority).  The default thread priority 
is 5 and this will be used if the priority is not explicitely set.

This funciton must be called before the thread is started.  Calling it
after the thread has already started will have no effect.

@param pri The thread's priority, in the range 0 to 9.
@return A valid error object.
*/
/***************************************************************************/
const Error *Thread::setPriority( int pri )
{
   if( pri < 0 || pri > 9 )
      return &ThreadError::BadParam;

   priority = pri;

   return 0;
}

/***************************************************************************/
/**
Start this thread.  The thread's virtual run() function will be called when
the thread has started.
*/
/***************************************************************************/
const Error *Thread::start( void )
{
   HANDLE h;
   const Error *err = 0;

   // Make sure my local data was properly allocated by the 
   // constructor.
   if( !data ) return &ThreadError::Alloc;

   // Allocate a single slot of thread local storage if not already done.
   globalThreadMutex.Lock();
   if( tlsIndex == TLS_OUT_OF_INDEXES )
      tlsIndex = TlsAlloc();

   if( tlsIndex == TLS_OUT_OF_INDEXES )
      err = &ThreadError::Alloc;
   globalThreadMutex.Unlock();
   if( err ) return err;

   WinThreadData *tData = (WinThreadData*)data;
   tData->mtx.Lock();

   // Make sure the thread isn't already started
   if( tData->running )
      err = &ThreadError::Running;

   else if( (h = (HANDLE)_beginthread( ThreadStarter, 0, tData )) == (HANDLE)-1 )
      err = &ThreadError::Start;

   else
   {
      int pri;
      switch( priority )
      {
	 case 0:  pri = THREAD_PRIORITY_IDLE;          break;
	 case 1:  pri = THREAD_PRIORITY_LOWEST;        break;
	 case 2:  pri = THREAD_PRIORITY_LOWEST;        break;
	 case 3:  pri = THREAD_PRIORITY_BELOW_NORMAL;  break;
	 case 4:  pri = THREAD_PRIORITY_BELOW_NORMAL;  break;
	 case 5:  pri = THREAD_PRIORITY_NORMAL;        break;
	 case 6:  pri = THREAD_PRIORITY_ABOVE_NORMAL;  break;
	 case 7:  pri = THREAD_PRIORITY_ABOVE_NORMAL;  break;
	 case 8:  pri = THREAD_PRIORITY_HIGHEST;       break;
	 case 9:  pri = THREAD_PRIORITY_TIME_CRITICAL; break;
	 default: pri = THREAD_PRIORITY_NORMAL;        break;
      }

      SetThreadPriority( h, pri );
      tData->running = true;
   }

   tData->mtx.Unlock();
   return err;
}

/***************************************************************************/
/**
Stop the thread.  The threads stack should unwind properly when stopped.
Note, I believe that there are race conditions if the thread returns and 
another thread trys to stop it at the same time.  Also make sure that 
things are properly cleaned up when the thread returns.
I may need to add a mutex to the thread object to help manage this.
*/
/***************************************************************************/
const Error *Thread::stop( int32 timeout )
{
   cml.Debug( "Stopping thread %p\n", this );

   if( !data ) return &ThreadError::Alloc;

   WinThreadData *tData = (WinThreadData *)data;
   tData->mtx.Lock();

   // Just return if the thread wasn't running
   if( !tData->running )
   {
      tData->mtx.Unlock();
      return 0;
   }
   tData->mtx.Unlock();

   // If this happens to by the running thread, then throw an exception.
   // This will cause my stack to unwind until the exception is caught
   // by the starter function.
   if( tData == GetThreadData() )
   {
      cml.Debug( "Thread %p is running thread, exiting\n", this );
      throw ThreadExitException();
   }

   // Otherwise, signal the thread to kill itself
   SetEvent( tData->killEvent );

   if( !timeout ) return 0;

   // Wait for the thread to exit
   DWORD ret = WaitForSingleObject( tData->exitEvent, timeout );

   switch( ret )
   {
      case WAIT_OBJECT_0:
	 cml.Debug( "Thread %p successfully stopped\n", this );
	 break;

      case WAIT_TIMEOUT:
	 cml.Warn( "Thread %p, timeout waiting for exit\n", this );
	 return &ThreadError::Timeout;

      default:
	 cml.Warn( "Thread %p, error %d waiting for exit\n", this, ret );
	 break;
   }

   return 0;
}

/***************************************************************************/
/**
Thread starting function.  This static function is called when a new thread
is started.  It calls Thread::run()
*/
/***************************************************************************/
static void __cdecl ThreadStarter( void *arg )
{
   WinThreadData *tData = (WinThreadData *)arg;
   TlsSetValue( tlsIndex, tData );

   try
   {
      tData->thread->run();
   }
   catch( ThreadExitException )
   {
   }

   // Set an event used to indicate that this thread is exited
   tData->mtx.Lock();
   SetEvent( tData->exitEvent );
   tData->running = false;
   tData->mtx.Unlock();

   return;
}

/***************************************************************************/
/**
Put the thread to sleep for a specified amount of time.
@param timeout The time to sleep in milliseconds.  If <0 the thread will
       sleep forever.
*/
/***************************************************************************/
const Error *Thread::sleep( int32 timeout )
{
   if( !timeout ) return 0;

   // Create a semaphore that no-one will ever post to
   // and wait on that with a timeout
   Semaphore sem = 0;

   const Error *err = sem.Get( timeout );

   // Convert timeout errors to no error since
   // that's what we expect to see.
   if( err == &ThreadError::Timeout )
      err = 0;

   return err;
}

/***************************************************************************/
/**
Return the current time in millisecond units
*/
/***************************************************************************/
uint32 Thread::getTimeMS( void )
{
   FILETIME ft;
   LARGE_INTEGER li;

   GetSystemTimeAsFileTime( &ft );
   li.LowPart = ft.dwLowDateTime;
   li.HighPart = ft.dwHighDateTime;

   return (uint32)(li.QuadPart/10000);
}


/***************************************************************************/
/**
Return a pointer to the running thread's WinThreadData structure.
@return A pointer to the structure, or NULL if the running thread wasn't
       created using the Thread object.
*/
/***************************************************************************/
static WinThreadData *GetThreadData( void )
{
   if( tlsIndex == TLS_OUT_OF_INDEXES ) return 0;
   return (WinThreadData *)TlsGetValue( tlsIndex );
}

/***************************************************************************/
/**
Create a new mutex object with default attributes.
*/
/***************************************************************************/
Mutex::Mutex( void )
{
   data = CreateMutex( NULL, FALSE, NULL );
}

/***************************************************************************/
/**
Create a new mutex object with default attributes.
*/
/***************************************************************************/
Mutex::~Mutex()
{
   if( data )
   {
      CloseHandle( (HANDLE)data );
   }
}

/***************************************************************************/
/**
Lock this mutex
*/
/***************************************************************************/
const Error *Mutex::Lock( void )
{
   if( !data ) return &ThreadError::Alloc;

   return WaitOnWindowsObject( (HANDLE)data, -1 );
}

/***************************************************************************/
/**
Unlock the mutex
*/
/***************************************************************************/
const Error *Mutex::Unlock( void )
{
   if( !data ) return &ThreadError::Alloc;

   if( !ReleaseMutex( (HANDLE)data ) )
      cml.Error( "ReleaseMutex failed with error %ld\n", GetLastError() );
   return 0;
}

/***************************************************************************/
/**
Default constructore for a semaphore object.  Initializes the semaphore to
it's default attributes.
*/
/***************************************************************************/
Semaphore::Semaphore( int32 count )
{
   data = CreateSemaphore( NULL, count, 0x7fffffff, NULL );
}

/***************************************************************************/
/**
Semaphore destructor
*/
/***************************************************************************/
Semaphore::~Semaphore()
{
   if( data )
      CloseHandle( (HANDLE)data );
}

/***************************************************************************/
/**
Get the semaphore with an optional timeout.  An error is returned if the 
timeout expires before the semaphore is acquired.
@param timeout The timeout in milliseconds.  Any negative value will cause
       the thread to wait indefinitely.
@return An error code indicating success or failure.
*/
/***************************************************************************/
const Error *Semaphore::Get( int32 timeout )
{
   if( !data ) return &ThreadError::Alloc;

   return WaitOnWindowsObject( (HANDLE)data, timeout );
}

/***************************************************************************/
/**
Post to the semaphore.
*/
/***************************************************************************/
const Error *Semaphore::Put( void )
{
   if( !data ) return &ThreadError::Alloc;

   if( !ReleaseSemaphore( (HANDLE)data, 1, NULL ) )
      return 0;
   return &ThreadError::General;
}

/***************************************************************************/
/**
Kill the running thread.
*/
/***************************************************************************/
static void KillThread( WinThreadData *tData )
{
   throw ThreadExitException();
}

/***************************************************************************/
/**
Wait for some Windows object (semaphore, mutex, event, etc) with a timeout.
This function should be used for Thread objects that need to wait on 
Windows objects rather then the standard WaitOnSingleObject call.  The 
reason is that this function allows the thread to be destroyed using Thread::stop.
Normally, windows objects aren't directly used anyway, but it's sometimes
required when writing the low level CAN driver for example.

@param hndl The handle to a windows object
@param timeout A timeout value (milliseconds).  Use -1 for infinite.
@return A pointer to an error object, or NULL on success
*/
/***************************************************************************/
const Error *WaitOnWindowsObject( HANDLE hndl, int32 timeout )
{
   DWORD ret;

   if( timeout < 0 ) timeout = INFINITE;

   // See if the calling thread was created using a Thread object.
   // If so, I will watch to make sure it isn't killed.
   WinThreadData *tData = GetThreadData();

   // If that returned NULL, then this thread isn't part of a Thread object.
   // In that case, just wait on the semaphore
   if( !tData )
   {
      ret = WaitForSingleObject( hndl, timeout );
   }

   // If this is part of a Thread object, then wait for either the semaphore
   // or the kill event.  If the kill event is set, then I'll exit the thread
   // gracefully.
   else
   {
      HANDLE h[2];
      h[0] = hndl;
      h[1] = tData->killEvent;

      ret = WaitForMultipleObjects( 2, h, FALSE, timeout );
   }

   switch( ret )
   {
      case WAIT_OBJECT_0:
	 return 0;

      case WAIT_OBJECT_0+1: 
	 KillThread( tData );
	 return 0;

      case WAIT_TIMEOUT:
	 // The wait timed out.  If a non-zero timeout was specified
	 // then try another wait with a zero timeout.  Windows seems 
	 // to generate false timeout conditions fairly often.
	 if( (timeout > 0) && (WaitForSingleObject( hndl, 0 ) == WAIT_OBJECT_0) )
	    return 0;
	 return &ThreadError::Timeout;

      default:
	 return &ThreadError::General;
   }
}

/***************************************************************************/
/**
This function can be used by a thread to check to see if it has been stopped.
Normally, the thread is automatically stopped when it tries to get a mutex 
or semaphore, but this can be used to stop a thread that is doing something
else for a long time.

It's presently just used in some CanInterface drivers for those drivers that
don't allow me to stop the thread in a more direct way.
*/
/***************************************************************************/
void CheckWindowsThreadStop( void )
{
   // Find my thread data.
   WinThreadData *tData = GetThreadData();

   // If that returned NULL, then I'm not part of a Thread object.
   // I can just return.
   if( !tData ) return;

   // Test the kill event for this thread.  Return if it's not signaled.
   if( WaitForSingleObject( tData->killEvent, 0 ) != WAIT_OBJECT_0 )
      return;

   // End it all!
   KillThread( tData );
}

