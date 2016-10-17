/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file

This file defines the top level error class used throughout the 
library.
*/

#ifndef _DEF_INC_ERROR
#define _DEF_INC_ERROR

#include "CML_ErrorCodes.h"
#include "CML_Settings.h"
#include "CML_Utils.h"

CML_NAMESPACE_START()

/**
This class is the root class for all error codes
returned by functions defined within the Motion Library.

Every error condition defined in the library has a constant,
static error object associated with it.  Pointers to these 
objects are returned from the various function calls.  

All library functions that return an error object pointer
will return a NULL pointer in the case of no error.  This
allows one to simply test the error pointer returned to
determine if it indicates an error.

For example:

<PRE>
	const Error *err = SomeFunctionCall();
	
	if( err )
		printf( "Error: %%s\\n", err->toString() );
	else
		printf( "no error\\n" );
		
</PRE>

To test for a specific error condition, the following code
can be used:

<PRE>
	const Error *err = SomeFunctionCall();
	
	if( err == &ThreadError::Timeout )
		printf( "A timeout occurred\\n" );
</PRE>

Note that the constructor used to create a new unique error
code is protected, therefore only sub-classes of the Error
object are allowed to create new unique error codes.
*/
class Error
{
public:
   /// A constant error object that represents no error.
   static const Error OK;

   /// An invalid error ID code was passed to Error::Lookup
   static const Error Unknown;

   /**
     Return a C style string describing the error condition
     represented by this error object.

     @return A constant character string describing the error
     condition.
     */
   const char *toString() const 
   {
#ifdef CML_ERROR_MESSAGES
      return description; 
#else
      return "Error descriptions not supported";
#endif
   }

   /***************************************************************************/
   /**
     Return an integer ID that can be used to identify the error.

     Each error code in the system has a unique 16-bit integer identifier associated
     with it.  This function can be used to return this identifier.

     These ID codes are primarily intended for internal use by the Error object, they
     are only provided externally for use in system which require an integer error ID
     code to interface with other parts of the system.  The function Error::Lookup
     can be used to convert the ID value back into an error object.

     @return The 16-bit integer ID value associated with this Error object.
     */
   /***************************************************************************/
   uint16 GetID( void ) const { return id; }

#if CML_ERROR_HASH_SIZE > 0 
   static const Error *Lookup( int16 id );
#endif

protected:
   /**
     Constructor used to create an error object of a particular
     type.  This constructor is protected, so only sub-classes 
     of the Error class can construct objects in this manner.

     A unique ID coded and a description string must be provided
     with the new object.
     */
   Error( uint16 i, const char *desc ): 
      id(i)
#ifdef CML_ERROR_MESSAGES
	 , description(desc)
#endif
   {
#if CML_ERROR_HASH_SIZE > 0 
      InitErrorHash();
#endif
   }

private:

   /// An ID code that identifies the type of the error.
   uint16 id;

   /// A constant description of the error code assigned
   /// to new error types.
#ifdef CML_ERROR_MESSAGES
   const char *description;
#endif

#if CML_ERROR_HASH_SIZE > 0 
   /// Used for error code lookup
   const Error *next;

   /// Internal function used to initialize the error object
   /// hash when a new error is constructed.
   void InitErrorHash( void );
#endif

   /// Private copy constructor (not supported)
   Error( const Error &e ){}

   /// Private assignment operator (not supported)
   Error &operator=( const Error &e ){ return *this;}
};

// The following macro is useful for creating new error objects
#ifdef CML_ERROR_MESSAGES
#  define CML_NEW_ERROR( group, name, message ) const group group::name( CMLERR_ ## group ## _ ## name, message )
#else
#  define CML_NEW_ERROR( group, name, message ) const group group::name( CMLERR_ ## group ## _ ## name, 0 )
#endif

CML_NAMESPACE_END()

#endif

