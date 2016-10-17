/********************************************************/
/*                                                      */
/*  Copley Motion Libraries                             */
/*                                                      */
/*  Copyright (c) 2002 Copley Controls Corp.            */
/*                     http://www.copleycontrols.com    */
/*                                                      */
/********************************************************/

/** \file
This file handles initializing the static data objects used by the Error class.
*/

#include "CML.h"

CML_NAMESPACE_USE();

CML_NEW_ERROR( Error, OK,      "No error" );
CML_NEW_ERROR( Error, Unknown, "Unknown error ID code" );

#if CML_ERROR_HASH_SIZE > 0 
// Error hash table
static const Error *errHash[ CML_ERROR_HASH_SIZE ];

/***************************************************************************/
/**
Lookup the constant error object associated with the passed ID code.
If the passed ID doesn't correspond to any known Error object, then
the address of the Error::Unknown will be returned.

@param id The ID code of the error to be found
@return A pointer to an error object.
*/
/***************************************************************************/
const Error *Error::Lookup( int16 id )
{
	// Look the error object up in the hash table
	const Error *err = errHash[ id % CML_ERROR_HASH_SIZE ];

	// Search all the errors in this hash location
	while( err && (err->id != id) )
		err = err->next;

	// If the error wasn't found, return 'unknown'
	if( !err )
	{
		cml.Error( "Unable to locate error id: %d\n", id );
		return &Error::Unknown;
	}

	return err;
}

/***************************************************************************/
/**
Initialize the Error object hash table.
*/
/***************************************************************************/

void Error::InitErrorHash( void )
{
	int i = id % CML_ERROR_HASH_SIZE;
	next = errHash[i];
	errHash[i] = this;
}

#endif

