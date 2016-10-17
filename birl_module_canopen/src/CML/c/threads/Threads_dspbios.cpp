/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

#include "CML.h"
#include <std.h>
#include <sem.h>
#include <tsk.h>

CML_NAMESPACE_USE();

/***********************************************************************
 * Do some quick sanity checks to make sure that the proper amount of
 * extra data has been provided in CML_Settings.h.
 * 
 * For use under DSPBIOS I need at least the following:
 *
 * CML_EXTRA_THREAD_DATA is not used
 * CML_EXTRA_MUTEX_DATA >= sizeof(SEM_Obj)
 * CML_EXTRA_SEMAPHORE_DATA >= sizeof(SEM_Obj)
 *
 * Right here I'll just check to make sure they are defined since the
 * preprocessor doesn't know about the sizeof operator.  I'll use 
 * assert statements below to double check.
 **********************************************************************/
#ifdef CML_EXTRA_THREAD_DATA
#  error CML_EXTRA_THREAD_DATA should be commented out in CML_Settings.h
#endif

#ifndef CML_EXTRA_MUTEX_DATA
#  error CML_EXTRA_MUTEX_DATA must be defined in CML_Settings.h
#endif

#ifndef CML_EXTRA_SEMAPHORE_DATA
#  error CML_EXTRA_SEMAPHORE_DATA must be defined in CML_Settings.h
#endif

/* local functions */
static int ThreadStarter( Thread *t );
static Uns ConvertTimeout( int32 timeout );

/***************************************************************************/
/**
Default constructor for a new thread object. 
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
	if( data )
		TSK_delete( (TSK_Handle)data );
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
	// Initialize this task's attributes to the defaults.
	TSK_Attrs attributes = TSK_ATTRS;

	// My priorities run from 0 to 9.  I'll map these to 
	// the DSP bios priorities 4 to 13
	attributes.priority = priority+4;

	TSK_Handle task = TSK_create( (Fxn)ThreadStarter, &attributes, this );
	if( !task )
		return &ThreadError::Alloc;

	data = (void*)task;
	return 0;
}

/***************************************************************************/
/**
Stop this thread.
*/
/***************************************************************************/
const Error *Thread::stop( int32 timeout )
{
	Uns to = ConvertTimeout( timeout );
	// FIXME: TBD
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
	Uns to = ConvertTimeout( timeout );

	// DSPBIOS doesn't support sleeping forever
	if( to == SYS_FOREVER )
		while( 1 ) TSK_sleep( 10000 );

	TSK_sleep( to );
	return 0;
}

/***************************************************************************/
/**
Create a new mutex object.
*/
/***************************************************************************/
Mutex::Mutex( void )
{
	CML_ASSERT( CML_EXTRA_MUTEX_DATA >= sizeof( SEM_Obj ) );

	SEM_Obj *sem = (SEM_Obj *)&data;
	SEM_new( sem, 1 );
}

/***************************************************************************/
/**
Destructor for mutex object.  
*/
/***************************************************************************/
Mutex::~Mutex( void )
{
}

/***************************************************************************/
/**
Lock this mutex
*/
/***************************************************************************/
const Error *Mutex::Lock( void )
{
	SEM_Obj *sem = (SEM_Obj *)&data;
	SEM_pend( sem, SYS_FOREVER );
	return 0;
}

/***************************************************************************/
/**
Unlock the mutex
*/
/***************************************************************************/
const Error *Mutex::Unlock( void )
{
	SEM_Obj *sem = (SEM_Obj *)&data;
	SEM_post( sem );
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
	CML_ASSERT( CML_EXTRA_MUTEX_DATA >= sizeof( SEM_Obj ) );
	CML_ASSERT( count >= 0 && count <= 32767 );

	SEM_Obj *sem = (SEM_Obj *)&data;
	SEM_new( sem, count );
}

/***************************************************************************/
/**
Destructor for Semaphore object.
*/
/***************************************************************************/
Semaphore::~Semaphore( void )
{
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
	SEM_Obj *sem = (SEM_Obj *)&data;
	Uns to = ConvertTimeout( timeout );

	return SEM_pend( sem, to ) ? 0 : &ThreadError::Timeout;	
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
	SEM_Obj *sem = (SEM_Obj *)&data;
	SEM_post( sem );
	return 0;
}

/***************************************************************************/
/**
 Convert a timeout value from my units (milliseconds) to DSPBIOS ticks.
*/
/***************************************************************************/
static Uns ConvertTimeout( int32 timeout )
{
	if( timeout < 0 )
		return SYS_FOREVER;

	// FIXME: Just cast for now
	return (Uns)timeout;
}

/***************************************************************************/
/**
 Thread starter.  This function calls the threads run method.
 */
/***************************************************************************/
static Int ThreadStarter( Thread *t )
{
	TSK_settime( TSK_self() );
	t->run();
	return 0;
}

