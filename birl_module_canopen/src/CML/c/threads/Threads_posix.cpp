/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "CML.h"
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

CML_NAMESPACE_USE();

/* local functions */
static void *ThreadStarter( void *arg );

/***************************************************************************/
/**
Default constructor for a new thread object.  For the pthreads version of
this class the default constructor doesn't do anything.
*/
/***************************************************************************/
Thread::Thread( void )
{
   priority = 5;
   data = 0;
}

/***************************************************************************/
/**
Destructor for a thread.  Causes the thread to be cancelled and waits for
it to finish.
*/
/***************************************************************************/
Thread::~Thread( void )
{
   stop(100);
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
   if( data ) return &ThreadError::Running;

   data = new pthread_t;
   if( !data ) return &ThreadError::Alloc;

   int min, max, inc;
   min = sched_get_priority_min( SCHED_FIFO );
   max = sched_get_priority_max( SCHED_FIFO );
   inc = (max-min+5)/10;

   struct sched_param sched;
   sched.sched_priority = min + priority * inc;

   if( sched.sched_priority < min ) sched.sched_priority = min;
   if( sched.sched_priority > max ) sched.sched_priority = max;

   pthread_attr_t attr;
   pthread_attr_init( &attr );
   pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED );
   pthread_attr_setschedparam( &attr, &sched );

   int ret = pthread_create( (pthread_t *)data, &attr, ThreadStarter, this );

   // If this fails, I'll try a second time with default attributes.  
   if( ret )
   {
      cml.Error( "pthread_create erorr %d creating a thread of pri %d.  Retrying...\n", ret, priority );
      ret = pthread_create( (pthread_t *)data, 0, ThreadStarter, this );
   }

   // If this still fails, I'll return an error.
   if( ret )
   {
      cml.Error( "pthread_create error %d creating a thread with default attributes.\n", ret );
      return &ThreadError::Start;
   }
   return 0;
}

/***************************************************************************/
/**
Stop this thread.
*/
/***************************************************************************/
const Error *Thread::stop( int32 timeout )
{
   if( data )
   {
      pthread_cancel( *(pthread_t *)data );
      delete (pthread_t *)data;
      data = 0;
   }
   return 0;
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
   Semaphore sem(0);

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
   struct timeval tv;
   gettimeofday( &tv, 0 );

   return (uint32)( tv.tv_sec*1000 + tv.tv_usec/1000 );
}

/***************************************************************************/
/**
Create a new mutex object.
*/
/***************************************************************************/
Mutex::Mutex( void )
{
   data = new pthread_mutex_t;

   if( data )
      pthread_mutex_init( (pthread_mutex_t *)data, NULL );
   else
      cml.Error( "Unable to allocate memory for mutex\n" );
}

/***************************************************************************/
/**
Destructor for mutex object.  
*/
/***************************************************************************/
Mutex::~Mutex( void )
{
   if( data )
   {
      pthread_mutex_destroy( (pthread_mutex_t *)data );
      delete (pthread_mutex_t *)data;
   }
}

/***************************************************************************/
/**
Lock this mutex
*/
/***************************************************************************/
const Error *Mutex::Lock( void )
{
   CML_ASSERT( data != 0 );
   if( !data ) return &ThreadError::Alloc;

   int err = pthread_mutex_lock( (pthread_mutex_t *)data );

   if( err )
      cml.Warn( "pthread_mutex_lock returned %d\n", err );

   return 0;
}

/***************************************************************************/
/**
Unlock the mutex
*/
/***************************************************************************/
const Error *Mutex::Unlock( void )
{
   CML_ASSERT( data != 0 );
   if( !data ) return &ThreadError::Alloc;

   int err = pthread_mutex_unlock( (pthread_mutex_t *)data );
   if( err )
      cml.Warn( "pthread_mutex_unlock returned %d\n", err );
   return 0;
}

/***************************************************************************/
/**
Default constructore for a semaphore object.  Initializes the semaphore to
it's default attributes.
@param count The initial count of the semaphore.  The semaphore's Get method
may be called that many times before any thread will block on the semaphore.
*/
/***************************************************************************/
Semaphore::Semaphore( int32 count )
{
   data = new sem_t;

   if( data )
      sem_init( (sem_t *)data, 0, count );
   else
      cml.Error( "Unable to allocate memory for semaphore\n" );
}

/***************************************************************************/
/**
Destructor for Semaphore object.
*/
/***************************************************************************/
Semaphore::~Semaphore( void )
{
   if( data )
   {
      sem_destroy( (sem_t *)data );
      delete (sem_t *)data;	
   }
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
   int err;

   CML_ASSERT( data != 0 );
   if( !data ) return &ThreadError::Alloc;

   if( timeout >= 0 )
   {
      struct timespec ts;

      err = clock_gettime( CLOCK_REALTIME, &ts );

      timeout += (ts.tv_nsec / 1000000);
      long sec = timeout / 1000;
      timeout -= sec * 1000;

      ts.tv_sec += sec;
      ts.tv_nsec = timeout * 1000000;

      err = sem_timedwait( (sem_t*)data, &ts );
   }
   else
   {
      err = sem_wait( (sem_t*)data );
   }

   if( err == 0 )
      return 0;

   if( err == -1 )
      err = errno;

   if( err == ETIMEDOUT )
      return &ThreadError::Timeout;

   return &ThreadError::General;
}

/***************************************************************************/
/**
Increase the count of the semaphore object.  If any threads are pending on 
the object, then the highest priority one will be made eligable to run.
@return An error object
*/
/***************************************************************************/
const Error *Semaphore::Put( void )
{
   CML_ASSERT( data != 0 );
   if( !data ) return &ThreadError::Alloc;

   if( sem_post( (sem_t*)data ) )
      return &ThreadError::General;

   return 0;
}

/// Static function used to start a new thread.
static void *ThreadStarter( void *arg )
{
   Thread *tptr = (Thread *)arg;
   tptr->run();
   return NULL;
}

