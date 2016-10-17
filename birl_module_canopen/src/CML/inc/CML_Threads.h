/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
The classes defined in this file provide an operating system independent
way of accessing multi-tasking system features.

The implementation of these classes will be different for different
supported platforms.
*/


#ifndef _DEF_INC_MULTITHREAD
#define _DEF_INC_MULTITHREAD

#include "CML_Settings.h"
#include "CML_Error.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
Errors related to the multi-threaded libraries.
*/
/***************************************************************************/
class ThreadError: public Error
{
public:
   /// Error starting the thread
   static const ThreadError Start;

   /// Thread has already been started
   static const ThreadError Running;

   /// Timeout waiting on semaphore
   static const ThreadError Timeout;	

   /// General failure
   static const ThreadError General;	

   /// Bad parameter passed to a thread function
   static const ThreadError BadParam;

   /// Error allocating memory for thread data
   static const ThreadError Alloc;

protected:
   ThreadError( uint16 id, const char *desc ): Error( id, desc ){}
};


/***************************************************************************/
/**
Virtual class which provides multi-tasking.  To add a new thread to a program,
create a new class that is derived from Thread.  The new thread of execution
will start when the start() member function is called.  This new thread of 
execution will start with the run() member function and will run concurrently
with the rest of the system.  When (if) the run method returns, the new thread
will be terminated.
*/
/***************************************************************************/
class Thread
{
   /// Private copy constructor (not supported)
   Thread( const Thread &t ){}

   /// Private assignment operator (not supported)
   Thread &operator=( const Thread &t ){ return *this;}

public:
   /// Create a new thread.  The new thread will not start executing
   /// until the start member function of this class is called.
   /// The default thread priority will be set to 5.
   Thread( void );

   /// Clean up any allocated resources
   virtual ~Thread();

   /// Set the thread priority.  This function should be called before
   /// the thread is started if the default priority (5) is not acceptable.
   /// @param pri The priority for this thread to run at.  The range is
   ///            0 to 9 where 0 is a very low priority task, and 9 is a 
   ///            critically high priority.
   /// @return An error object is returned indicating the success of the call.
   const Error *setPriority( int pri );

   /// Make this thread eligible to run.  The new thread will be created
   /// if possible and identified to the operating system as eligible to run.
   /// When the thread actually starts, the run() method will be called.
   /// Note that depending on the priority of the thread and of the calling 
   /// task, the run() function may or may not be called before start() returns.
   /// @return An error object is returned indicating the success of the call.
   const Error *start( void );

   /// Stop this thread.  The thread will have exited by the time this function
   /// returns.  If the calling thread is the thread being stopped, then this
   /// function will not return.
   /// @param timeout The amount of time to wait (milliseconds) for the thread
   ///        to stop before returning an error.
   /// @return An error object is returned indicating the success of the call.
   const Error *stop( int32 timeout=1000 );

   /// Cause the calling thread to sleep for a specified number of milliseconds.
   /// @param timeout The time to sleep, in milliseconds.
   /// @return An error object is returned indicating the success of the call.
   static const Error *sleep( int32 timeout );

   /// Return the current time in millisecond units.  The value returned may be 
   /// offset by a consistent, but arbitrary value.  This makes it useful for 
   /// checking relative times, but not useful for absolute time calculations.
   /// @return The time in millisecond units
   static uint32 getTimeMS( void );

   /// When a new thread is started, this function will be called.  All of the 
   /// thread specific code should be contained in this function.  If the run()
   /// method ever returns, the thread will be destroyed.
   virtual void run( void ) = 0;
private:

   /// Holds the threads priority.
   int priority;

   /// Holds system specific data necessary to implement the thread object.
#ifdef CML_EXTRA_THREAD_DATA
   byte data[ CML_EXTRA_THREAD_DATA ];
#else
   void *data;
#endif
};


/***************************************************************************/
/**
This class represents an object that can be used by multiple threads to gain
safe access to a shared resource.  If an attempt is made to lock a mutex that
is currently locked by another thread, the thread attempting the lock will be
suspended until the thread holding the lock releases it.
Mutex objects are not required to allow recursive access.
*/
/***************************************************************************/
class Mutex
{
   /// Private copy constructor (not supported)
   Mutex( const Mutex &m ){}

   /// Private assignment operator (not supported)
   Mutex &operator=( const Mutex &m ){ return *this; }

public:
   /// Create a new mutex object
   Mutex( void );

   /// Free any system resources associated with the mutex
   virtual ~Mutex();

   /// Lock the mutex.  This function causes the calling function to gain 
   /// exclusive access to the mutex object.  If some other thread has the
   /// mutex locked when this method is called, the calling thread will 
   /// block until the mutex is unlocked.
   /// @return An error object.
   const Error *Lock( void );

   /// Unlock the mutex.  This function causes the calling thread to give 
   /// up it's lock on the mutex.  A task switch may occur before this call
   /// returns if a high priority task is currently trying to lock the mutex.
   /// @return An error object.
   const Error *Unlock( void );	

private:
   /// Pointer to any system specific data necessary to implement the mutex.
#ifdef CML_EXTRA_MUTEX_DATA
   byte data[ CML_EXTRA_MUTEX_DATA ];
#else
   void *data;
#endif
};

/***************************************************************************/
/**
This is a utility class that locks a mutex in it's constructor, and unlocks 
it in it's destructor.  It can be used to ensure that a mutex is properly 
unlocked when a function returns.  Just create a temporary MutexLocker object
and pass it the mutex to lock in it's constructor.  The mutex will be automatically
unlocked when the function returns and the MutexLocker is deleted.
*/
/***************************************************************************/
class MutexLocker
{
   Mutex *mPtr;

public:
   /// Lock the passed mutex
   /// @param m The mutex to lock.
   MutexLocker( Mutex &m )
   {
      mPtr = &m;
      mPtr->Lock();
   }

   /// Unlock the mutex
   ~MutexLocker()
   {
      mPtr->Unlock();
   }
};

/***************************************************************************/
/**
Generic semaphore class.  Semaphores can be used to allow multiple threads
to share a pool of shared resources.  Semaphores can be used like mutexes,
however they also implement timeouts and multiple resource counts.
*/
/***************************************************************************/
class Semaphore
{
   /// Private copy constructor (not supported)
   Semaphore( const Semaphore &s ){}

   /// Private assignment operator (not supported)
   Semaphore &operator=( const Semaphore &s ){ return *this;}
public:
   /// Create a new semaphore object.  If a count is passed, then the initial 
   /// semaphore count will be initialized to that value.  If no count is passed, 
   /// then a count of zero is used.
   ///
   /// @param count The initial count of the semaphore.  The semaphore's Get method
   ///        may be called that many times before any thread will block on it.
   Semaphore( int32 count=0 );

   /// Free any system resources associated with this semaphore.  Any threads blocking
   /// on the semaphore should return from the Get() call with an error indication.
   virtual ~Semaphore();

   /// Get the semaphore with an optional timeout.  An error is returned if the 
   /// timeout expires before the semaphore is acquired.
   /// @param timeout The timeout in milliseconds.  Any negative value will cause
   ///        the thread to wait indefinitely.  If a timeout of zero is specified, 
   ///        the calling thread will return a timeout error without blocking if 
   ///        the semaphore is not available.
   /// @return An error code indicating success or failure.
   const Error *Get( int32 timeout=-1 );

   /// Increase the count of the semaphore object.  If any threads are pending on 
   /// the object, then the highest priority one will be made eligible to run.
   /// @return An error object
   const Error *Put( void );	

private:
	/// Pointer to any system specific data necessary to implement the semaphore.
#ifdef CML_EXTRA_SEMAPHORE_DATA
   byte data[ CML_EXTRA_SEMAPHORE_DATA ];
#else
   void *data;
#endif
};

CML_NAMESPACE_END()

#endif


