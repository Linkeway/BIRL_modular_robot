/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file defines the Event Map class.
*/

#ifndef _DEF_INC_EVENTMAP
#define _DEF_INC_EVENTMAP

#include "CML_Settings.h"
#include "CML_Threads.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/***************************************************************************/
/**
This class represents error conditions related to the Event object.
*/
/***************************************************************************/
class EventError: public Error
{
public:
   /// The event is already mapped to another object
   static const EventError AlreadyOwned;

   /// The event is not mapped to this object.
   static const EventError NotMapped;

protected:
   /// Standard protected constructor
   EventError( uint16 id, const char *desc ): Error( id, desc ){}
};

/***************************************************************************/
/**
Events are a generic mechanism used to wait on some condition.  They are
used in conjunction with the EventMap object.

Every EventMap object has a 32-bit mask register which describes it's state.
Event objects may be attached to an EventMap to wait for any combination of
bits in the mask to become active.

The base Event class is a virtual class, and therefore shouldn't be used
directy.  It is extended by a number of sub-classes which are used to wait
for certain map bits to be set, cleared, etc.

An Event object may only be assigned to one EventMap at a time.  An attempt
to attach it to multiple EventMap objects will result in an error.  Further, 
Event objects are not thread safe.  Only one thread should access a particular
Event object at a time.  EventMap objects however are thread safe, so any 
number of threads may attach their own Event objects to the same EventMap
object without issue.
*/
/***************************************************************************/
class Event
{
   friend class EventMap;

   Mutex mtx;

   /// Pointer to the map that this event is assigned to
   class EventMap *map;

   Event *prev, *next;

   /// If a semaphore is attached to the Event, then the
   /// semaphone will be posted each time the event becomes
   /// true.
   Semaphore *semPtr;

   /// Used for more complex event types
   class EventMap *chainMap;
   uint32 chainMask;

   /// This is the most recent mask that cause the event to succeed
   uint32 trueMask;

   /// Called by the EventMap for all attached Events 
   /// whenever the map's mask is updated.
   void update( uint32 newMask );

protected:
   /// This is the value that the event is waiting for
   uint32 value;

public:
   Event( uint32 val=0 );
   virtual ~Event();
   Event( const Event & );
   Event &operator=( const Event & );

   const Error *setValue( uint32 val );

   void setChain( class EventMap &map, uint32 mask );
   void delChain( void );

   /***************************************************************************/
   /**
     Return the value that this event will wait on.
     @return The event value
     */
   /***************************************************************************/
   uint32 getValue( void ){ return value; }

   /***************************************************************************/
   /**
     Return the most recent mask value that caused the event to succeed.
     After a successful Wait, this can be used to return the mask that 
     caused the successful match.
     */
   /***************************************************************************/
   uint32 getMask( void ){ return trueMask; }

   const Error *Wait( EventMap &m, int32 timeout );

   /***************************************************************************/
   /**
     Test the event to see if it's condition is true.  This method should be
     implemented in the sub-class to define the type of event matching used.
     The base class always returns false.

     @param mask The EventMap mask compare to.
     @return true if the event is satisfied, false if not.
     */
   /***************************************************************************/
   virtual bool isTrue( uint32 mask ){ return false; }
};

/***************************************************************************/
/**
This is an event that matches if any of a group of bits are set in the 
EventMap mask.
*/
/***************************************************************************/
class EventAny: public Event
{
public:
   /***************************************************************************/
   /**
     Construct a new object specifying a group of bits that should be checked.
     @param v The bit mask that the event will wait on.  Default is zero
     */
   /***************************************************************************/
   EventAny( uint32 v=0 ):Event(v) {}

   /**
     Construct a new event object using the value of a passed event.
     @param e Another event object who's value will be used to initialize this one.
     */
   EventAny( const EventAny &e ):Event(e){}

   /***************************************************************************/
   /**
     Check the event.  The event is satisfied if any of the selected bits are set
     in the mask.
     @return true if any bits of interest are set in the mask.
     */
   /***************************************************************************/
   virtual bool isTrue( uint32 mask )
   {
      return (mask & value) != 0;
   }
};

/***************************************************************************/
/**
This is an event that matches if any of a group of bits are clear in the 
EventMap mask.
*/
/***************************************************************************/
class EventAnyClear: public Event
{
public:
   /***************************************************************************/
   /**
     Construct a new object specifying a group of bits that should be checked.
     @param v The bit mask that the event will wait on.  Default is zero
     */
   /***************************************************************************/
   EventAnyClear( uint32 v=0 ):Event(v) {}
   
   /**
     Construct a new event object using the value of a passed event.
     @param e Another event object who's value will be used to initialize this one.
     */
   EventAnyClear( const EventAnyClear &e ):Event(e){}

   /***************************************************************************/
   /**
     Check the event.  The event is satisfied if any of the selected bits are clear
     in the mask.
     @return true if any bits of interest are set in the mask.
     */
   /***************************************************************************/
   virtual bool isTrue( uint32 mask )
   {
      return (mask & value) != value;
   }
};

/***************************************************************************/
/**
This is an event that matches if all of a group of bits are set in the 
EventMap mask.
*/
/***************************************************************************/
class EventAll: public Event
{
public:
   /***************************************************************************/
   /**
     Construct a new object specifying a group of bits that should be checked.
     @param v The bit mask that the event will wait on.  Default is zero
     */
   /***************************************************************************/
   EventAll( uint32 v=0 ):Event(v){}
   
   /**
     Construct a new event object using the value of a passed event.
     @param e Another event object who's value will be used to initialize this one.
     */
   EventAll( const EventAll &e ):Event(e){}

   /***************************************************************************/
   /**
     Check the event against the passed mask.  If all of the selected bits are set
     in the mask, then the event is satisfied.
     @param mask The mask to test against
     @return true if all bits of interest are set in the mask.
     */
   /***************************************************************************/
   virtual bool isTrue( uint32 mask )
   {
      return (mask & value) == value;
   }
};

/***************************************************************************/
/**
This is an event that matches if none of a group of bits are set in the 
EventMap mask.
*/
/***************************************************************************/
class EventNone: public Event
{
public:
   /***************************************************************************/
   /**
     Construct a new object specifying a group of bits that should be checked.
     @param v The bit mask that the event will wait on.  Default is zero
     */
   /***************************************************************************/
   EventNone( uint32 v=0 ):Event(v) {}

   /**
     Construct a new event object using the value of a passed event.
     @param e Another event object who's value will be used to initialize this one.
     */
   EventNone( const EventNone &e ):Event(e){}

   /***************************************************************************/
   /**
     Check the event against the passed mask.  If none of the selected bits are set
     in the mask, then the event is satisfied.
     @param mask The mask to test against
     @return true if none bits of interest are set in the mask.
     */
   /***************************************************************************/
   virtual bool isTrue( uint32 mask )
   {
      return (mask & value) == 0;
   }
};

/***************************************************************************/
/**
An event map is a mechanism that allows one or more threads to wait on some
pre-defined event, or group of events.

For a particular event map, there are one or more events that are grouped 
with that map.  Any number of threads may pend on the state of these events.
*/
/***************************************************************************/
class EventMap
{
   /// Private copy constructor (not supported)
   EventMap( const EventMap & );

   /// Private assignment operator (not supported)
   EventMap &operator=( const EventMap & );

public:
   EventMap()
   {
      mask = 0;
      list = 0;
   }

   virtual ~EventMap( void );
   const Error *Add( Event *e );
   const Error *Remove( Event *e );

   /***************************************************************************/
   /**
     Get the current value of the mask for this event map.
     @return The 32-bit mask value.
     */
   /***************************************************************************/
   uint32 getMask( void ){ return mask; }

   /***************************************************************************/
   /**
     Update the event mask.  The new mask value will equal the passed parameter.
     @param mask The new mask value.
     */
   /***************************************************************************/
   void setMask( uint32 mask )
   {
      mutex.Lock();
      update( mask );
      mutex.Unlock();
   }

   /***************************************************************************/
   /**
     Set bits in the event mask.  
     @param bits Any bit set in this parameter will be set in the event mask.
     */
   /***************************************************************************/
   void setBits( uint32 bits )
   {
      mutex.Lock();
      update( mask | bits );
      mutex.Unlock();
   }

   /***************************************************************************/
   /**
     Clear bits in the event mask.  
     @param bits Any bit set in this parameter will be cleared in the event mask.
     */
   /***************************************************************************/
   void clrBits( uint32 bits )
   {
      mutex.Lock();
      update( mask & ~bits );
      mutex.Unlock();
   }

   /***************************************************************************/
   /**
     Change the value of specified bits in the mask for this event.  The bits to
     change are identified by one parameter, and the new value for these bits
     is specified in the other parameter.
     @param bits Identifies which bits in the mask to change.  Only those bits
     which are set in this parameter will be effected in the event mask.
     @param value The new value for the bits identified in the first parameter.
     */
   /***************************************************************************/
   void changeBits( uint32 bits, uint32 value )
   {
      value &= bits;

      mutex.Lock();
      update( (mask & ~bits) | value );
      mutex.Unlock();
   }

private:
   Mutex mutex;
   uint32 mask;
   Event *list;

   void update( uint32 newMask );

   friend class Event;
};

CML_NAMESPACE_END()

#endif

